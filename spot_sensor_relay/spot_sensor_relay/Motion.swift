//import Foundation
//import CoreMotion
//
//
//let motionManager = CMMotionManager()
//
//
//if motionManager.isDeviceMotionAvailable {
//    // Start device motion updates
//    motionManager.startDeviceMotionUpdates(to: .main) { (data, error) in
//        guard let data = data else { return }
//
//        // Get the attitude (roll, pitch, yaw)
//        let attitude = data.attitude
//
//        // Print roll, pitch and yaw in radians
//        print("Roll: \(attitude.roll)")
//        print("Pitch: \(attitude.pitch)")
//        print("Yaw: \(attitude.yaw)")
//    }
//}
