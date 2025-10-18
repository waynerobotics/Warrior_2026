# Warrior_2026

``` mermaid
graph TD
    %% =========================
    %% INPUTS (AUGUSTINE)
    %% =========================
    subgraph Augustine [Augustine]
        Odom[Odometry / Encoders / IMU / GPS]
        EKF[Extended Kalman Filter]
        FiltOdom[Filtered Odometry (Best Guess Position)]

        Odom --> EKF --> FiltOdom
    end

    %% =========================
    %% CAMERA + LIDAR INPUTS
    %% =========================
    Camera --> RoboPercept[Robot Perception]
    Lidar --> PointCloud[Point Cloud]

    %% =========================
    %% AI & CLASSICAL PERCEPTION (SALV RYAN)
    %% =========================
    subgraph SalvRyan [Salv Ryan]
        ClassPercept[Classical Perception]
        AIRecog[AI Perception (Solve: MoveUs)]
        ClassPercept --> AIRecog
    end

    RoboPercept --> ClassPercept
    PointCloud --> AIRecog

    AIRecog --> Mask

    %% =========================
    %% SENSOR FUSION (JENNIFER)
    %% =========================
    subgraph Jennifer [Jennifer]
        Mask --> SensorFusion[Sensor Fusion]
        SensorFusion --> CostMap[Cost Map]
    end

    %% =========================
    %% NAVIGATION (COLIN)
    %% =========================
    subgraph Colin [Colin]
        CostMap --> Nav2Solve[Nav2 / AI Solve (MoveUs)]
    end

    Nav2Solve --> CmdVel[Cmd-Vel]

    %% =========================
    %% OUTPUT CONTROL (SALEM)
    %% =========================
    subgraph Salem [Salem]
        CmdVel --> Arduino[Arduino]
    end

    %% =========================
    %% SIMULATION CONTEXTS
    %% =========================
    classDef sim fill:#d3f9d8,stroke:#2d7a2d,stroke-width:1px;
    classDef real fill:#ffe6cc,stroke:#ff6600,stroke-width:1px;

    subgraph AreasOfInterest [Areas of Interest]
        Real[Real: Arduino → Real Robot]
        Gazebo[Simulation: Gazebo (ROS2 Solve)]
        IsaacSim[Simulation: IsaacSim (AI Solve)]
    end

    %% =========================
    %% NOTES / COLORS
    %% =========================
    %% Augustine = Red, Salv Ryan = Blue, Jennifer = Orange, Colin = Red, Salem = Red
    %% Gazebo = Green (servo drive / diff drive → AI)

    style Augustine fill:#ffd6d6,stroke:#ff0000
    style SalvRyan fill:#d6e0ff,stroke:#0000ff
    style Jennifer fill:#ffeccf,stroke:#ff6600
    style Colin fill:#ffd6d6,stroke:#ff0000
    style Salem fill:#ffd6d6,stroke:#ff0000
```
