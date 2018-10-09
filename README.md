# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## PID Controller

The “p”(proportional) parameter directs the steering value to be proportional and opposite to how far and in what direction the car is away from the centre of the road. 

Setting only this value makes the car to oscillate a lot and doesn’t settle to the centre of the road. On curved roads, the car moves out of the road due to these oscillations.

The “d”(differential) parameter reduces the oscillation of the “p” controller by reducing the steering value based on the difference in the last and current cross-track-error(cte). The “pd” controller performs much better than the “p” controller.

The “I”(integral) parameter helps to add little more to the steering value to handle scenarios like bias in the steering wheel alignment. However, this value was set to very small in this project and didn’t see major difference in behaviour due to this parameter compared to “p” and “d” parameters.

## Parameter Tuning

The control parameters were tuned using a combination of manual tuning and twiddle. Referred to the projects of some of the senior students(like jeremy-shannon, sohonisaurabh) to get an idea on the tuned parameters and twiddle implementation. Those values were tuned to fit to the controller. 

There were many situations in which the car moved out of the road during turns. To handle this scenario, implemented a logic to set throttle value of zero whenever the steering values are out of the given threshold range. The basic concept of this and the below logic is taken from the Udacity's knowledge forum.

Whenever the car doesn’t move much due to continuously varying steering angles between opposite extremes or the throttle value set to zero in the above use case defined in above paragraph, I’ve implemented logic to push the vehicle forward for few steps on a given steering angle and throttle value.

I could notice that the car moves slowly in some of the turns. The push forward logic kicks in during such situations and move the car forward.

Video below shows the final result of the PID controller -> https://youtu.be/Ntdez9y3ioM


<iframe width="560" height="315" src="https://www.youtube.com/embed/Ntdez9y3ioM" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>



