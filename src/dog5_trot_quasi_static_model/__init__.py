"""dog5_trot_quasi_static_model -- the trot that runs on DOG5, in five files.

    config      every constant, no logic
    gait        the contact clock: which diagonal is down, and the load ramp
    trot_hw     the runner.  CROUCH -> WAIT -> RISE -> HOLD -> TROT -> PARK
    trot_demo   trot_hw plus a fixed four-foot settle every N cycles
    walk_demo   trot_demo plus a step displacement: out on T, back on R

WHAT IS AND IS NOT IN HERE
    The FORCE path is not.  trot_hw imports it from `torque_stand/`, which is
    the week-2 stand -- the same wrench law, the same grasp map, the same
    joint impedance and the same TorqueGate that have been up and down on this
    machine.  What this package adds to a stand is a clock and a swing arc.

    A trot that cannot stand first is not a trot problem, and rewriting the
    part that already works is how the previous attempt ended up unable to
    rise at all.  So the split is: `torque_stand/` holds everything that was
    already flying, and everything NEW is here.

WHY IT IS "QUASI-STATIC"
    The body model is one rigid body with the acceleration terms sent to zero
    -- no I*alpha, no omega x (I omega).  Hand it a wrench and it returns foot
    forces, every tick, with no memory.  For a trot IN PLACE the trunk's
    angular acceleration is small and both dropped terms are small with it;
    for a trot that TRAVELS they are not, and this package is honest that it
    does not have them.  See docs/ch4_quasi_dynamic_trot.md.

IMPORT IT AS A PACKAGE.  `config` is a very common module name and this repo
already has its own at the top level, which motorbus.py imports.  Putting this
directory on sys.path shadows it and breaks the CAN layer with an unrelated
AttributeError several imports later.  Every module here therefore imports its
siblings through the package, and `dog5_paths` keeps this one directory OFF
sys.path so the collision is structurally impossible rather than merely
commented against.  `selftest/test_layout.py` is the standing gate.
"""
