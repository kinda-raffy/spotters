//import SwiftUI
//import CoreMotion
//import Network
//
//
//class CameraManager: NSObject {
//    
//    var connection: NWConnection?
//   let hostUDP: NWEndpoint.Host = "192.168.20.87"  // Change this to your desired IP.
//   let portUDP: NWEndpoint.Port = 7068
//    
//    override init() {
//        super.init()
//        startUDPSocket()
//    }
//    
//    private func startUDPSocket() {
//        self.connection = NWConnection(host: hostUDP, port: portUDP, using: .udp)
//
//        self.connection?.stateUpdateHandler = { (newState) in
//            print("This is stateUpdateHandler:")
//            switch (newState) {
//                case .ready:
//                    print("State: Ready\n")
//                    self.receiveUDP()
//                case .setup:
//                    print("State: Setup\n")
//                case .cancelled:
//                    print("State: Cancelled\n")
//                case .preparing:
//                    print("State: Preparing\n")
//                default:
//                    print("ERROR! State not defined!\n")
//            }
//        }
//
//        self.connection?.start(queue: .global())
//    }
//    
//    func sendUDP(_ content: Data) {
//        self.connection?.send(content: content, completion: NWConnection.SendCompletion.contentProcessed(({ (NWError) in
//            if (NWError == nil) {
//                print("Data was sent to UDP")
//            } else {
//                print("ERROR! Error when data (Type: Data) sending. NWError: \n \(NWError!)")
//            }
//        })))
//    }
//    
//    func receiveUDP() {
//        self.connection?.receiveMessage { (data, context, isComplete, error) in
//            if (isComplete) {
//                print("Receive is complete")
//                if (data != nil) {
//                    let backToString = String(decoding: data!, as: UTF8.self)
//                    print("Received message: \(backToString)")
//                } else {
//                    print("Data == nil")
//                }
//            }
//        }
//    }
//
//}
//
//
//struct ContentView: View {
//    private let cameraManager = CameraManager()
//    private let motionManager = CMMotionManager()
//    
//    @State private var roll: Double = 0
//    @State private var pitch: Double = 0
//    @State private var yaw: Double = 0
//    @State private var rotationRate: CMRotationRate = CMRotationRate(x: 0, y: 0, z: 0)
//    @State private var userAcceleration: CMAcceleration = CMAcceleration(x: 0, y: 0, z: 0)
//
//    var body: some View {
//        VStack {
//           Image(systemName: "antenna.radiowaves.left.and.right.circle")
//                .foregroundColor(.accentColor)
//                .padding(5)
//                .font(.system(size: 40))
//                .symbolRenderingMode(.hierarchical)
//            Text("Broadcasting")
//                .foregroundColor(.gray)
//                .padding(1)
//           Text("\(getWiFiAddress() ?? "N/A"):80/rgb")
//                .font(.system(.body, design: .monospaced))
//                .bold()
//                .padding(5)
//            Text("IMU")
//                .foregroundColor(.gray)
//                .padding(2)
//            Text("Roll: \(roll)")
//            Text("Pitch: \(pitch)")
//            Text("Yaw: \(yaw)")
//            Text("Rotation Rate (X: \(rotationRate.x), Y: \(rotationRate.y), Z: \(rotationRate.z))")
//            Text("User Acceleration (X: \(userAcceleration.x), Y: \(userAcceleration.y), Z: \(userAcceleration.z))")
//        }
//        .padding()
//        .onAppear {
//            startMotionUpdates()
//        }
//        .onDisappear {
//            motionManager.stopDeviceMotionUpdates()
//        }
//    }
//    
//    func startMotionUpdates() {
//        motionManager.deviceMotionUpdateInterval = 1.0 / 10.0  // hz.
//        if motionManager.isDeviceMotionAvailable {
//                    motionManager.startDeviceMotionUpdates(to: .main) { (data, error) in
//                    guard let data = data else { return }
//                        
//                    let motionDataString: String = "your motion data as string"
//                    if let data = motionDataString.data(using: .utf8) {
//                        cameraManager.sendUDP(data)
//                    }
//                        
//                    roll = data.attitude.roll
//                    pitch = data.attitude.pitch
//                    yaw = data.attitude.yaw
//                    rotationRate = data.rotationRate
//                    userAcceleration = data.userAcceleration
//                }
//            }
//        }
//
//    func getWiFiAddress() -> String? {
//        var address : String?
//
//        var ifaddr: UnsafeMutablePointer<ifaddrs>?
//        guard getifaddrs(&ifaddr) == 0 else { return nil }
//        guard let firstAddr = ifaddr else { return nil }
//
//        for ptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
//            let flags = Int32(ptr.pointee.ifa_flags)
//            let addr = ptr.pointee.ifa_addr.pointee
//            if (flags & (IFF_UP|IFF_RUNNING|IFF_LOOPBACK)) == (IFF_UP|IFF_RUNNING) {
//                if addr.sa_family == UInt8(AF_INET) {
//                    if let name = ptr.pointee.ifa_name,
//                       let network = String(cString: name, encoding: .utf8),
//                       network == "en2" {  // Device's internal ip address.
//                        var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
//                        getnameinfo(ptr.pointee.ifa_addr, socklen_t(addr.sa_len), &hostname, socklen_t(hostname.count),
//                                    nil, socklen_t(0), NI_NUMERICHOST)
//                        address = String(cString: hostname)
//                    }
//                }
//            }
//        }
//
//        freeifaddrs(ifaddr)
//        return address
//    }
//}
//
//struct ContentView_Previews: PreviewProvider {
//    static var previews: some View {
//        ContentView()
//    }
//}
