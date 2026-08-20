# V1.4.2 C++ -> pure C mapping

| C++ V1.4.2 | Pure-C module |
|---|---|
| `cv::Mat gray` zero-copy VI view | `GrayImage` (`data/width/height/stride`) |
| `cv::Mat previous_gray` | owned `U8Image previous_gray` |
| `cv::Mat background_q8` | owned `U16Image background_q8` |
| `cv::Mat` ROI/blob masks | owned `U8Image` |
| `cv::mean` | `gray_mean()` |
| `std::vector<int>` flood queue | one-time `malloc(width*height*sizeof(int))` |
| `std::vector<MotionBlob>` | one-time allocated `MotionBlobList` |
| `enum class` | C `typedef enum` |
| references | pointers |
| `std::chrono::steady_clock` | `clock_gettime(CLOCK_MONOTONIC)` |
| `std::chrono::system_clock` | `clock_gettime(CLOCK_REALTIME)` |
| `CameraPipeline` class | `CameraPipeline` struct + `camera_pipeline_*()` functions |
| Event/Snapshot C++ ring structs | fixed-size C ring structs |

## Memory ownership

`GrayImage` never owns VI memory. It is valid only before `RK_MPI_VI_ReleaseChnFrame()`.

Persistent buffers are allocated once after the first frame establishes width/height:

- Q8 background: `width*height*2`
- previous Y: `width*height`
- gully/static/blob masks: 3 x `width*height`
- flood queue: `width*height*sizeof(int)`
- MotionBlob list: bounded by `pixel_count / 120 + 32`

There is no per-frame `malloc/free` in the normal processing path.

## Deliberately unchanged V1.4.2 behavior

The conversion does not redesign thresholds or state machines. In particular it preserves:

- outside-gully camera-motion threshold = 3%
- dominant replacement factor = 4x
- DATA update = 1 second
- IMAGE capture = device-global 30 seconds
- event close/merge grace = 3 seconds
- 8-neighbor connected components
- PGM snapshots and existing `/tmp` file names
- `[MOTION]`, `[BLOB]`, `[TRACK]`, `[QUEUE-DATA]`, `[QUEUE-IMAGE]` field names/order
