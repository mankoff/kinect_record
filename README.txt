
== File: README.txt
== Author: Ken Mankoff
== Date: May 2012

== Purpose:

This code is a combination of the OpenKinect libfreenect 'glview' and
'record' programs. The purpose is to let you see the data you are
recording. 

This code offers some improvements over the default 'record' program:

* On-screen real-time feedback and display of the data being recorded
* Does not auto-level the Kinect
* Allows the folder to be used multiple times (over-writes INDEX.txt)

== Installation:

Install the latest libfreenect, then build this by typing
"make" on OS X or "make -f Makefile.linux" or Linux. You might need to
adjust some parameters in the Makefile.

== Usage:

./kinect_record <folder>

To stop recording, press ESC when the display window has the focus.
