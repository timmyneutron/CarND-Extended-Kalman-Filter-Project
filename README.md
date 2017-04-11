![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)

# Udacity Self-Driving Car Nanodegree: Term 2
# Project #1: Extended Kalman Filter

## Introduction
This is a project for Udacity's Self-Driving Car Nanodegree. It implements an Extended Kalman Filter to estimate a car's state based on radar and laser measurements.


## Concepts and Classes
Concepts explored in this project:

  - Kalman Filters and Extended Kalman Filters
  - Radar and laser sensor fusion

Relevant classes:

  - [Artificial Intelligence for Robotics](https://classroom.udacity.com/courses/cs373)

## Getting Started
Code is stored in the `src` directory.

Input data and output data (state estimations and final root mean squared error) are in the `data` directory.

To run the code, navigate to the `build` directory and execute the following:

`./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output1.txt`

or

`./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt output2.txt`
