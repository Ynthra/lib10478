Pretty sure there's undefined behaviour somewhere btw so don't use this code 💀.
Also my fault for the inconsistent formatting / naming.
# 
Check [here](https://github.com/Ynthra/lib10478/tree/183daaee0ecbc13e4baaf4a6dc1e99542810931a) for completed, working version (what was used at nationals)

# Dependencies
- [Lemlib/hardware](https://github.com/LemLib/hardware)
- [Lemlib/units](https://github.com/LemLib/units)
- [Lemlib Eigen port](https://github.com/LemLib/Eigen)
# Resources used
- [coopers rough (and broken) motion profiling implementation](https://github.com/Cooper7196/Real-Time-Motion-Profiling)
- [wpilib](https://github.com/wpilibsuite/allwpilib) for RAMSETE implementation
- [this book](https://controls-in-frc.link/) for RAMSETE explanation
- deepseek for vibecoding the whole path planner 🐐
- [this](https://github.com/Ryan4253/ryanlib) and [this](https://github.com/joshua-jose/23218A-TippingPoint-Public) codebases for pros multithreading stuff

# TODO
- [ ] finish main chassis task
- [ ] add quintic bezier support with c2 continuity
- [ ] fix path planner importing standalone bezier curves
- [ ] find potential UB so I'm not screwed at worlds
- [ ] support flipping autons properly
