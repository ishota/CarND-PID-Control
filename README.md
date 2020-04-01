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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Result

My best parameters is here.
```cpp
double kp = 0.07;     //p gain
double ki = 0.000073; //i gain
double kd = 0.507;    //d gain
```

[demo]: https://user-images.githubusercontent.com/13342802/77847282-03a18800-71f7-11ea-8153-ac0408317db9.gif

Result video is here.

![result gif][demo]

## Control setting

I set throttle value from 0.3 to 0.2.
```cpp
double throttle_value = 0.2;
```

I set the maximum value of the steering control amount as follows.
```cpp
if ( steer > 1 ) {
  steer = 0.99;
}
if ( steer < -1 ) {
  steer = -0.99;
}
```

## Parameter serach

### Manual search

#### P control

The control amount increases or deccreases in proportion to the difference from the target value.
In case of the project simulation, the further away from the center of the lane, the bigger the difference.

The relationship between a P gain and the behavior of the car is described below.

- When the P gain is small, the control amount toward the center of the lane is small, and the steady error is large.
- Increasing P gain reduces the steady error but overshoots lane center.
- Does not converge if the P gain is too large.


#### D control

The control amount is determined based on the differential value of the difference from the target value.
In case of the project simulation, as the amount of change in the distance from the lane center increases, a D gain increases.

- When the P gain is large and does not converge, the conergence is achieved by giving the D gain.


#### I control

The control amount is determined based on the integrated value of the difference from the target value.
In case of the project simulation, the control amount is determined based on a value obtained by integrating the difference between the distance from the lane center.

- By increasing the I gain, the steady error can be suppressed.

### Twidle search

I implemented the twiddle parameter search.

Manually adjusted parameters were set first.
Using the application that operates the GUI with the timer, the simulator was restarted and a twiddle search was perfomed in toe following procedure.
Basically, it left the parameters longer.


```cpp
//TWIDDLE
if ( (fabs(cte) > bad_cte || sim_time > good_time ) && escape_sw == 0) {
  twigle_iteration++;
  escape_sw = 1;

  // sim_time is over good_time
  if ( sim_time > good_time ) {

    // best_goodness
    if ( goodness < best_goodness ){
      printf(" godness %f < best_goodness %f", goodness, best_goodness);
      best_goodness = goodness;
      pre_goodness = best_goodness;
      pre_sim_time = sim_time;
      best_kp = kp;
      best_ki = ki;
      best_kd = kd;
    } else {
    // not best
      printf(" godness %f >= best_goodness %f", goodness, best_goodness);
      pre_goodness = best_goodness;
      pre_sim_time = best_sim_time;
      kp = best_kp;
      ki = best_ki;
      kd = best_kd;
    }

  // sim_time is lower than best time
  } else if ( sim_time > best_sim_time ) {
    printf(" sim_time %d > best_sim_time %d", sim_time, best_sim_time);
    best_sim_time = sim_time;
    pre_goodness = goodness;
    best_goodness = goodness;
    pre_sim_time = sim_time;
    best_kp = kp;
    best_ki = ki;
    best_kd = kd;
  } else if ( pre_sim_time > sim_time ){
  // sim_time is lower than previous sim_time
    printf(" pre_sim_time %d > sim_time %d", pre_sim_time, sim_time);
    twigle_sub_count++;
    kp = pre_kp;
    ki = pre_ki;
    kd = pre_kd;
    if ( twigle_sub_count % 2 != 0 ) {
      twigle_count++;
    }
  } else {
    pre_goodness = goodness;
    pre_sim_time = sim_time;
  }

  pre_kp = kp;
  pre_ki = ki;
  pre_kd = kd;

  printf(" tw:%d %d %d, good:%f(%f), sim_time:%d(%d), ",twigle_iteration, twigle_count, twigle_sub_count, goodness, best_goodness, sim_time, best_sim_time);
  double x = (double)rand()/RAND_MAX;
  x /= 10;

  if ( twigle_count % 3 == 0 ) {
    if ( twigle_sub_count % 2 == 0 ) {
      std::cout << " ki up:" << ki << " -> ";
      ki *= 1 + x;
      std::cout << ki;
    } else {
      std::cout << " ki down:" << ki << " -> ";
      ki *= 1 - x;
      std::cout << ki;
    }
  } else if (twigle_count % 2 == 0 ) {
    if ( twigle_sub_count % 2 == 0 ) {
      std::cout << " kd up:" << kd << " -> ";
      kd *= 1 + x;
      std::cout << kd;
    } else {
      std::cout << " kd down:" << kd << " -> ";
      kd *= 1 - x;
      std::cout << kd;
    }
  } else {
    if ( twigle_sub_count % 2 == 0 ) {
      std::cout << " kp up:" << kp << " -> ";
      kp *= 1 + x;
      std::cout << kp;
    } else {
      std::cout << " kp down:" << kp << " -> ";
      kp *= 1 - x;
      std::cout << kp;
    }
  }

  // pid coef update
  printf("\n");
}

```



