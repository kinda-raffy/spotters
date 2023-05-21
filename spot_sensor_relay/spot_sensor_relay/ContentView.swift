import AVFoundation
import SwiftUI
import Swifter
import ImageIO
import MobileCoreServices
import UniformTypeIdentifiers

class CameraManager: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    private let captureSession = AVCaptureSession()
    private let encodeQueue = DispatchQueue(label: "EncodeQueue")
    private var webServer: HttpServer?
    @Published var imageData: Data? = nil
    
    override init() {
        super.init()
        setupCamera()
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
        
        // Set the resolution
        captureSession.sessionPreset = .vga640x480  // .hd1280x720
    }
    
    private func startSession() {
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            self?.captureSession.startRunning()
        }
    }
    
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let cvPixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        let ciImage = CIImage(cvPixelBuffer: cvPixelBuffer)
        guard let cgImage = convertCIImageToCGImage(inputImage: ciImage) else { return }
        
        encodeQueue.async {
            // Convert CGImage to JPEG.
            let imageData = self.convertCGImageToJPEG(cgImage: cgImage)
            // Update imageData.
            DispatchQueue.main.async {
                self.imageData = imageData
            }
        }
    }
    
    private func convertCIImageToCGImage(inputImage: CIImage) -> CGImage? {
        let context = CIContext(options: nil)
        return context.createCGImage(inputImage, from: inputImage.extent)
    }
    
    private func convertCGImageToJPEG(cgImage: CGImage) -> Data? {
        let mutableData = CFDataCreateMutable(nil, 0)
        guard let dataOut = CGDataConsumer(data: mutableData!) else { return nil }
        guard let imageDestination = CGImageDestinationCreateWithDataConsumer(dataOut, kUTTypeJPEG, 1, nil) else { return nil }
        
        let imageProperties = [
            kCGImageDestinationLossyCompressionQuality: 0.75
        ] as CFDictionary
        
        CGImageDestinationAddImage(imageDestination, cgImage, imageProperties)
        CGImageDestinationFinalize(imageDestination)
        
        return mutableData as Data?
    }
    
    private func startWebServer() {
        let server = HttpServer()
        
        server["/rgb"] = { [weak self] request in
            return .raw(200, "OK", ["Content-Type": "multipart/x-mixed-replace; boundary=--boundary"], { writer in
                while let data = self?.imageData {
                    let jpegHeader = "--boundary\r\n" +
                        "Content-Type: image/jpeg\r\n" +
                        "Content-Length: \(data.count)\r\n\r\n"
                    try? writer.write(jpegHeader.data(using: .utf8)!)
                    try? writer.write(data)
                    try? writer.write("\r\n".data(using: .utf8)!)
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
