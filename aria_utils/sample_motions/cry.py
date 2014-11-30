#!/usr/bin/env python
import rospy
import time
import aria

def cry():
	aria.set_interpolation('sigmoid')
	aria.eye([0.0,-1.0,1.0])
	aria.set_interpolation('bezier')
	time.sleep(0.1)
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.3823007345199585,0.04188790172338486,1.0890854597091675,-2.094395160675049,0.0,0.6283185482025146,0.0,0.0,-0.2932153046131134,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	aria.set_interpolation('slowin')
	time.sleep(1.5)
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.7120943069458008,-0.2702064275741577,1.2985249757766724,-1.9279644918441772,0.9634217619895935,0.6283185482025146,0.25,0.0,-0.2932153046131134,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	aria.set_interpolation('bezier')
	time.sleep(0.6)
	aria.set_positions([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.2932153046131134,0.0,0.6702064275741577,-0.46076691150665283,0.9634217619895935,0.6283185482025146,0.0,0.0,-0.2932153046131134,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])
	time.sleep(1.0)

if __name__ == '__main__':
	aria.init_publisher()
	rospy.init_node('aria_motion')
	cry()
