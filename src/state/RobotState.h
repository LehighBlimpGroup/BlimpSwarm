


class RobotState {
public:
    virtual ~RobotState() {}
    // Pure virtual function to update state based on sensors and controls
    virtual RobotState* update(float sensors[], float controls[], float outControls[]) = 0;

};
