# Pipeline and architecture

polka collapses a multi-node fusion chain into one composable node, cutting latency, CPU overhead, and configuration complexity. Managing several LiDARs in ROS 2 normally means a chain of separate nodes, each adding overhead, latency, and a failure point.

## What polka replaces

### polka (1 node)

```mermaid
graph LR
    subgraph Drivers
        D1[lidar driver · front]
        D2[odom / cmd_vel]
        D3[lidar driver · back]
    end

    P[<strong>polka</strong>]

    subgraph Consumers
        C1[mapping / reconstruction<br/>~/merged_cloud]
        C2[localization / navigation<br/>~/merged_scan]
    end

    D1 --> P
    D2 -.-> P
    D3 --> P
    P --> C1
    P --> C2
```

### pcl_ros chain (7+ nodes)

Cloud path:

```mermaid
graph LR
    subgraph Drivers
        D1[lidar driver · front]
        D2[lidar driver · back]
    end

    CAT[pcl_ros::<br/>ConcatenatePointCloud<br/>+ ApproxTimeSynchronizer]
    CF[custom node<br/>cloud filters]
    MAP[mapping node]

    D1 --> CAT
    D2 --> CAT
    CAT --> CF -->|merged_cloud| MAP
```

Scan path:

```mermaid
graph LR
    subgraph Drivers
        D1[lidar driver · front]
        D2[lidar driver · back]
    end

    P2L1[pointcloud_to_laserscan<br/>· front]
    P2L2[pointcloud_to_laserscan<br/>· back]
    IRA[ira_laser_tools::<br/>LaserscanMerger]
    SF[custom node<br/>scan filters]
    NAV[localization / navigation]

    D1 --> P2L1
    D2 --> P2L2
    P2L1 --> IRA
    P2L2 --> IRA
    IRA --> SF -->|merged_scan| NAV
```

## Internal stages

```mermaid
graph LR
    subgraph Sources
        PC[PointCloud2<br/>/front/points]
        LS[LaserScan<br/>/rear/scan]
    end

    subgraph Per-Source Filters
        PF1[Range / Angular /<br/>Box Filter]
        PF2[Range / Angular /<br/>Box Filter]
    end

    subgraph Merge Engine
        ME[CPU or CUDA<br/>Merge]
    end

    subgraph Output Pipeline
        OF[Range / Angular /<br/>Box Filter]
        FF[Footprint Filter]
        HF[Height Filter]
        VX[Voxel Downsample]
    end

    PC --> PF1 --> ME
    LS --> PF2 --> ME
    ME --> OF --> FF --> HF --> VX
    VX --> OUT_PC[PointCloud2]
    VX --> OUT_LS[LaserScan]
```

Each source is filtered in its own frame before the merge, so irrelevant data is dropped early. The merge engine (CPU or CUDA) transforms every source into `output_frame_id` and concatenates. The output pipeline then applies the shared filters, height cap, footprint exclusion, and voxel downsample in a fixed order before publishing the cloud and/or the flattened scan.

## File structure

```
polka/
├── config/
│   ├── example_params.yaml           # Minimal starter config
│   ├── detailed_params.yaml          # Full annotated parameter reference
│   └── example_articulated_imu.yaml  # Per-source IMU deskewing example
├── launch/polka.launch.py            # Launch file
├── doc/                              # docs + assets, one folder
│   ├── CONFIGURATION.md
│   ├── PIPELINE.md
│   ├── images/polka.png              # logo
│   └── media/                        # demo GIFs (gifs/) + generator toolchain
├── include/polka/
│   ├── polka_node.hpp                # Main composable node (orchestration only)
│   ├── types.hpp                     # Config structs and type definitions
│   ├── config/config_loader.hpp      # Parameter loading and hot-reload
│   ├── input/
│   │   ├── source_adapter.hpp        # Subscribes to and converts sensor data
│   │   └── imu_buffer.hpp            # IMU ring buffer with atomic snapshot
│   ├── filters/
│   │   ├── i_filter.hpp              # Filter interface
│   │   ├── filter_chain.hpp          # Factory: build a filter chain from FilterParams
│   │   ├── range_filter.hpp          # Min/max distance filter
│   │   ├── angular_filter.hpp        # Angular sector filter
│   │   └── box_filter.hpp            # Axis-aligned box filter (invert for self filter)
│   ├── merge_engine/
│   │   ├── i_merge_engine.hpp        # Merge engine interface
│   │   ├── cpu_merge_engine.hpp      # CPU merge implementation
│   │   ├── cuda_merge_engine.hpp     # CUDA GPU merge implementation
│   │   └── cuda_types.cuh            # GPU type definitions
│   ├── output/
│   │   ├── output_pipeline.hpp       # Post-merge processing (filter, height cap, voxel)
│   │   └── scan_builder.hpp          # LaserScan assembly from cloud or range vector
│   └── util/
│       ├── qos_builder.hpp           # build_qos() for output publishers
│       ├── se3_exp.hpp               # SE(3) exponential map for motion compensation
│       └── log_format.hpp            # Log throttle constants
└── src/
    ├── main.cpp                      # Entry point
    ├── polka_node.cpp                # Node implementation
    ├── config_loader.cpp
    ├── source_adapter.cpp
    ├── imu_buffer.cpp
    ├── filters/                      # Filter implementations
    ├── merge_engine/                 # Merge engine implementations
    └── output/                       # OutputPipeline and ScanBuilder implementations
```
