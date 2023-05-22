import AVFoundation
import SwiftUI
import Swifter
import VideoToolbox


class CameraManager: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    private let captureSession = AVCaptureSession()
    private var compressionSession: VTCompressionSession?
    // Note: self.image data is being read and written to by different threads.
    private let imageDataQueue = DispatchQueue(label: "ImageDataQueue")

    private var webServer: HttpServer?
    @Published var imageData: Data? = nil
    
    override init() {
        super.init()
        setupCamera()
        setupCompressionSession()
        startWebServer()
    }
    
    private func setupCamera() {
        guard let camera = AVCaptureDevice.default(for: .video) else {
            print("No camera available")
            return
        }
        
        do {
            let cameraInput = try AVCaptureDeviceInput(device: camera)
            configureCamera(camera)
            addInputAndOutputToSession(input: cameraInput)
            startSession()
        } catch {
            print("Failed to setup camera: \(error)")
        }
    }
    
    private func configureCamera(_ camera: AVCaptureDevice) {
        do {
            try camera.lockForConfiguration()
            // Framerate.
            camera.activeVideoMinFrameDuration = CMTime(value: 1, timescale: Int32(30))
            camera.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: Int32(30))
            camera.unlockForConfiguration()
        } catch {
            print("Failed to lock camera for configuration: \(error)")
        }
    }
    
    private func addInputAndOutputToSession(input: AVCaptureDeviceInput) {
        if captureSession.canAddInput(input) {
            captureSession.addInput(input)
        }
        
        let videoOutput = AVCaptureVideoDataOutput()
        videoOutput.setSampleBufferDelegate(self, queue: DispatchQueue(label: "sample buffer delegate", attributes: []))
        
        if captureSession.canAddOutput(videoOutput) {
            captureSession.addOutput(videoOutput)
        }
        
        // Resolution.
        captureSession.sessionPreset = .vga640x480  // .hd1280x720
    }
    
    private func startSession() {
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            self?.captureSession.startRunning()
        }
    }
    
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let cvPixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }

        if let compressionSession = compressionSession {
            var flagsOut: VTEncodeInfoFlags = VTEncodeInfoFlags()
            let status = VTCompressionSessionEncodeFrame(compressionSession, imageBuffer: cvPixelBuffer, presentationTimeStamp: CMTime.invalid, duration: CMTime.invalid, frameProperties: nil, sourceFrameRefcon: nil, infoFlagsOut: &flagsOut)

            if status != noErr {
                print("Failed to encode frame: \(status)")
            }
        } else {
            print("Compression session is not set up")
        }
    }
    
    private func setupCompressionSession() {
        let width = 640
        let height = 480

        let status = VTCompressionSessionCreate(allocator: kCFAllocatorDefault,
                                                width: Int32(width),
                                                height: Int32(height),
                                                codecType: kCMVideoCodecType_JPEG,
                                                encoderSpecification: nil,
                                                imageBufferAttributes: nil,
                                                compressedDataAllocator: nil,
                                                outputCallback: { outputCallbackRefCon, sourceFrameRefcon, status, infoFlags, sampleBuffer in
                                                    guard status == noErr else {
                                                        print("Error in outputCallback: \(status)")
                                                        return
                                                    }
                                                    guard let sampleBuffer = sampleBuffer else {
                                                        print("sampleBuffer was null in outputCallback")
                                                        return
                                                    }
                                                    guard let outputCallbackRefCon = outputCallbackRefCon else {
                                                        print("outputCallbackRefCon was null in outputCallback")
                                                        return
                                                    }
                                                    let cameraManager: CameraManager = Unmanaged.fromOpaque(outputCallbackRefCon).takeUnretainedValue()
                                                    cameraManager.handleOutput(sampleBuffer: sampleBuffer)
                                                },
                                                refcon: UnsafeMutableRawPointer(Unmanaged.passUnretained(self).toOpaque()),
                                                compressionSessionOut: &compressionSession)

        guard status == noErr else {
            print("Failed to create compression session: \(status)")
            return
        }

        VTSessionSetProperty(compressionSession!, key: kVTCompressionPropertyKey_RealTime, value: kCFBooleanTrue)
    }
    
    private func handleOutput(sampleBuffer: CMSampleBuffer) {
        guard let dataBuffer = CMSampleBufferGetDataBuffer(sampleBuffer) else { return }
        
        var length: Int = 0
        var dataPointer: UnsafeMutablePointer<Int8>?
        let status = CMBlockBufferGetDataPointer(dataBuffer, atOffset: 0, lengthAtOffsetOut: nil, totalLengthOut: &length, dataPointerOut: &dataPointer)
        
        guard status == kCMBlockBufferNoErr else {
            print("Failed to get data pointer from sample buffer: \(status)")
            return
        }
        
        let data = Data(bytes: dataPointer!, count: length)
        imageDataQueue.async(flags: .barrier) {
            self.imageData = data
        }
    }
    
    private func startWebServer() {
        let server = HttpServer()
        // This looks ugly.
        server["/rgb"] = { [weak self] request in
            return .raw(200, "OK", ["Content-Type": "multipart/x-mixed-replace; boundary=--boundary"], { writer in
                while true {  // Everything in the imageData queue.
                    self?.imageDataQueue.sync {  // Obtain lock.
                        if let data = self?.imageData {  // Publish image.
                            let jpegHeader = "--boundary\r\n" +
                                "Content-Type: image/jpeg\r\n" +
                                "Content-Length: \(data.count)\r\n\r\n"
                            try? writer.write(jpegHeader.data(using: .utf8)!)
                            try? writer.write(data)
                            try? writer.write("\r\n".data(using: .utf8)!)
                        }
                    }
                }
            })
        }
        
        do {
            try server.start(6969)
        } catch {
            print("Server start error: \(error)")
        }
        
        self.webServer = server
    }
}


struct ContentView: View {
    private var cameraManager = CameraManager()

    var body: some View {
        VStack {
           Image(systemName: "antenna.radiowaves.left.and.right.circle")
                .foregroundColor(.accentColor)
                .padding(5)
                .font(.system(size: 40))
                .symbolRenderingMode(.hierarchical)
            Text("Broadcasting")
                .foregroundColor(.gray)
                .padding(1)
           Text("\(getWiFiAddress() ?? "N/A"):6969/rgb")
                .font(.system(.body, design: .monospaced))
                .bold()
       }
       .padding()
    }

    func getWiFiAddress() -> String? {
        var address : String?

        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&ifaddr) == 0 else { return nil }
        guard let firstAddr = ifaddr else { return nil }

        for ptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
            let flags = Int32(ptr.pointee.ifa_flags)
            let addr = ptr.pointee.ifa_addr.pointee
            if (flags & (IFF_UP|IFF_RUNNING|IFF_LOOPBACK)) == (IFF_UP|IFF_RUNNING) {
                if addr.sa_family == UInt8(AF_INET) {
                    if let name = ptr.pointee.ifa_name,
                       let network = String(cString: name, encoding: .utf8),
                       network == "en2" {  // Device's internal ip address.
                        var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                        getnameinfo(ptr.pointee.ifa_addr, socklen_t(addr.sa_len), &hostname, socklen_t(hostname.count),
                                    nil, socklen_t(0), NI_NUMERICHOST)
                        address = String(cString: hostname)
                    }
                }
            }
        }

        freeifaddrs(ifaddr)
        return address
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
