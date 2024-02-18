# LEGO Gyroboy -- My Way

My take on LEGO's gyroboy project for the EV3.

<iframe width="560" height="315" src="https://www.youtube.com/embed/sPfzWnu0Gz4?si=SmgIr4S0IDI0JnN1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

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
project. You can see the contortions I had to do to get the PID controller to run
fast enough. Even with all this effort one time through the loop takes an average of
7ms on my EV3. (The demo code from LEGO goes faster, but, of course, it's doing a lot
less.)

It was a brilliantly fun project and a great diversion for a week of evenings. Looking
forward to the next one.