# Issues #

[Is there constrained inverse dynamics?](https://github.com/stack-of-tasks/pinocchio/issues/1052)

The answer is no. Authors suggest to look at [TSID](https://github.com/stack-of-tasks/tsid) library.

[Is there inverse dynamics for floating base systems?](https://github.com/stack-of-tasks/pinocchio/issues/1343)

The answer is no. Authors suggest to look at [TSID](https://github.com/stack-of-tasks/tsid) library.

[What is the difference between frame and joint Jacobian? What do functions `getJointJacobian` and `computeJointJacobian` exactly do?](https://github.com/stack-of-tasks/pinocchio/issues/1455)

# Functions #

The function `forwardDynamics(model, data, q)` fills `oMf` array, each of its elements is transformation from frame to origin.
Are u sure?

```c++
struct Model
{
    /// @brief Dimension of configuration space
    int nq;

    /// @brief Number of degrees of freedom
    int nv;

    /// @brief Number of joints
    int njoints;

    /// @brief Array of names of joints
    std::vector<std::string> names;
};
```
