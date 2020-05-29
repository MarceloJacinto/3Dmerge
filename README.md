# 3DMerge
This project consists in merging point clouds . In addition we prodive a very simple mechanism to detect moving objects between the 3D images.

## Scope of the project
This project was developed in the scope of the UC of Image Processing and Vision at IST (University of Lisbon). This UC was organized by professors João Paulo Costeira and José Santos Vitor.

The dataset used in this project was provided by professor João Paulo Costeira in is personal website:
http://printart.isr.ist.utl.pt/piv/datasets/

A full report written in Portuguese is also provided explaining the thought process behind the project (report.pdf). **Unfortunaly this report is not available in English.**

## Recommendation
In order to better understand the topics covered in this project we recommend the following website:
- https://medium.com/machine-learning-world/linear-algebra-points-matching-with-svd-in-3d-space-2553173e8fed

We also recommend searching for the topic: homography.

## Running
In order to run this project one just needs to run the main.m file in MATLAB. This file shows visually all the steps the algorithm goes through in order to obtain the final results. It is recommended to run the code by sections in order to better understand each step.

It is also provided the merge3d.m file which provides a function that makes all the calculations presented in main.m. The goal is to provide the same results in a reusable way. The merge3d.m file contains all the functions needed to work inside, unlike main.m which needs all the auxiliary functions provided.

**NOTE:** In order to run this project, please read the external libraries requirements.

## External Libraries
- VLFeat - to make use of SIFT features
- get_rgbd.m and get_xyzasus.m - code made available by professor João Paulo Costeira used to match points captured by the RGB camera 

## Known bugs
There are some known issues when running this code in MATLAB versions previous to 2019.

This project is by no means perfect and could be improved in a million different ways! Feel free to improve it!

## Developed by:
- Marcelo Jacinto
- David Souto