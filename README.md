# A Linear N-Point Solver for Structure and Motion from Asynchronous Tracks (ICCV 2025 Highlight)

<p align="center">
  <img src="https://img.shields.io/badge/ICCV_2025-Highlight-orange" alt="ICCV 2025 Highlight">
  <a href="https://arxiv.org/abs/2507.22733" target="_blank"><img src="https://img.shields.io/badge/arXiv-2507.22733-b31b1b.svg" alt="arXiv"></a>
  <a href="[# (Link to Video Presentation - Coming Soon)]" target="_blank"><img src="https://img.shields.io/badge/Video-Coming_Soon-lightgrey?logo=youtube" alt="Video"></a>
  <a href="[# (Link to Poster - Coming Soon)]" target="_blank"><img src="https://img.shields.io/badge/Poster-Coming_Soon-lightgrey" alt="Poster"></a>
</p>

<p align="center">
<strong>Hang Su<sup>1</sup>, Yunlong Feng<sup>1</sup>, Daniel Gehrig<sup>2</sup>, Panfeng Jiang<sup>1</sup>, Ling Gao<sup>3</sup>, Xavier Lagorce<sup>1</sup>, Laurent Kneip<sup>1</sup></strong>
</p>

<p align="center">
<sup>1</sup>ShanghaiTech University &nbsp;&nbsp;&nbsp; <sup>2</sup>University of Pennsylvania &nbsp;&nbsp;&nbsp; <sup>3</sup>Amap, Alibaba Group
</p>

---

> **Abstract**
> *Structure and continuous motion estimation from point correspondences is a fundamental problem in computer vision that has been powered by well-known algorithms such as the familiar 5-point or 8-point algorithm. However, despite their acclaim, these algorithms are limited to processing point correspondences originating from a pair of views each one representing an instantaneous capture of the scene. Yet, in the case of rolling shutter cameras, or more recently, event cameras, this synchronization breaks down. In this work, we present a unified approach for structure and linear motion estimation from 2D point correspondences with arbitrary timestamps, from an arbitrary set of views. By formulating the problem in terms of first-order dynamics and leveraging a constant velocity motion model, we derive a novel, linear point incidence relation allowing for the efficient recovery of both linear velocity and 3D points with predictable degeneracies and solution multiplicities. Owing to its general formulation, it can handle correspondences from a wide range of sensing modalities such as global shutter, rolling shutter, and event cameras, and can even combine correspondences from different collocated sensors. We validate the effectiveness of our solver on both simulated and real-world data, where we show consistent improvement across all modalities when compared to recent approaches. We believe our work opens the door to efficient structure and motion estimation from asynchronous data.*

<p align="center">
  <img src="assets/teaser.png" alt="Teaser Image" width="50%">
  <br>
  <em>A linear N-point solver for recovering 3D points and the velocity of a camera undergoing quasi-linear motion, given a set of timestamped observations</em>
</p>