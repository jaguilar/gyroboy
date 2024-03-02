# LEGO Gyroboy -- My Way

My take on LEGO's gyroboy project for the EV3.

Current, with C PID controller:

https://github.com/jaguilar/gyroboy/assets/799564/d14a14c5-e32a-4bad-8059-a4c31950a997

Before, with Micropython-Viper PID controller:

https://github.com/jaguilar/gyroboy/assets/799564/7a09d942-e44e-4272-b6c3-2ca7feb5c35d

Yeah, I mean, it's okay? I made it "controllable" by a PC controller (see
control.py). The control is not very responsive and is prone to causing things
to fall apart whenever you let go of the stick. Also sometimes it doesn't go
in the direction you want it to. I feel like I've played with it as much as I
feel like at this point and it's time to try something else.

Part of me wonders of PID is just not the right control model for a robot like
this. I'd like to think so except there are other people who have managed to get
things working. If I had to guess, my error metric should get a linearizing transform.
Instead of angle, I should probably be reporting imputed torque on the motor as the
value to be controlled. The only problem is that at least in EV3, we don't actually
know which way is up, so we can't do the math to find out the torque. But maybe we
could estimate it over time? Maybe by looking at what happens when a given level of
motor power is applied to a given angle?

I also found that pybricks was a little slow of a programming language for this
project. I ended up rewriting the PID controller in C. (See [pid.c].) I found that this
made the robot MUCH steadier. The average loop time went from 7-8ms to 5ms. Presumably
the variance in the loop times is also much better controlled.

It was a brilliantly fun project and a great diversion for a week of evenings. Looking
forward to the next one.
