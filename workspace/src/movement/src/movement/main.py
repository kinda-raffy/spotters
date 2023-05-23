from spot_driver.spot_ros import SpotROS


def main():
    SR = SpotROS()
    SR.main()
    return "Main movement called"
