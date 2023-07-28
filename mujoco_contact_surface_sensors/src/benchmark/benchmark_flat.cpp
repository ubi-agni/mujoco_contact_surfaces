#include <mujoco_contact_surface_sensors/flat_tactile_sensor.h>

#include <mujoco_contact_surfaces/plugin_utils.h>
#include <mujoco_contact_surfaces/mujoco_contact_surfaces_plugin.h>
#include <ros/package.h>

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros/array_safety.h>

#include <iostream>
#include <fstream>

using namespace mujoco_ros::contact_surfaces;
using namespace mujoco_ros::contact_surfaces::sensors;
namespace mju = ::mujoco::sample_util;

#ifdef USE_SSE
static const bool sse_on = true;
#else
static const bool sse_on = false;
#endif

struct Stats {
    Stats(std::string v) : variant(v) {}
    std::string variant;
    double bblas, btlas, nblas, tr, h, total, nrays, ntri, scollision, sevaluate, spassive;
};

struct Config {
    Config(bool use_p, float res, int sample_res, std::string m) : use_parallel(use_p), resolution(res), sampling_resolution(sample_res), mesh(m) {}
    void print_config() {
        ROS_INFO_STREAM("Running config with:\n\tuse_parallel: " << use_parallel << "\n\tresolution: " << resolution << "\n\tsampling_resolution: " << sampling_resolution << "\n\tmesh: " << mesh);
    }
    bool use_parallel = false;
    float resolution = 0.0;
    int sampling_resolution = 0;
    std::string mesh;
    bool use_sse = sse_on;
};

struct Entry {
    Entry(Config c, Stats s1, Stats s2, Stats s3) : config(c), stats{s1, s2, s3} {}
    Config config;
    Stats stats[3];
};

struct BenchResult {
    void to_csv(std::string path) {
        std::fstream file;
        file.open(path, std::ios::out);
        file << "use_sse,use_parallel,resolution,sampling_resolution,mesh,surface_collision,surface_passive,surface_evaluate,algorithm,bblas,btlas,nblas,tr,h,total,nrays,ntri\n";
        for (Entry e : entries) {
            for (Stats s : e.stats) {
                file << e.config.use_sse << "," << 
                e.config.use_parallel << "," << 
                e.config.resolution << "," << 
                e.config.sampling_resolution << 
                "," << e.config.mesh << 
                "," << s.scollision <<
                "," << s.spassive <<
                "," << s.sevaluate <<
                "," << s.variant << 
                "," << s.bblas << "," << 
                s.btlas << "," << 
                s.nblas << "," << 
                s.tr << "," << 
                s.h << "," << 
                s.total << "," << 
                s.nrays << "," << 
                s.ntri << "\n";
            }
        }
        file.close();
    }
    std::vector<Entry> entries;
};


std::string read_string_from_file(const std::string &file_path) {
    const std::ifstream input_stream(file_path, std::ios_base::binary);

    if (input_stream.fail()) {
        throw std::runtime_error("Failed to open file");
    }

    std::stringstream buffer;
    buffer << input_stream.rdbuf();

    return buffer.str();
}

void set_ros_config(Config &config) {
    XmlRpc::XmlRpcValue sensor_params;
    sensor_params.setSize(1);
    sensor_params[0]["type"] = "mujoco_contact_surface_sensors/FlatTactileSensor";
    sensor_params[0]["sensorName"] = "myrmex_sensor0";
    sensor_params[0]["geomName"] = "myrmex_foam";
    sensor_params[0]["topicName"] = "/tactile_module_16x16_v2";
    sensor_params[0]["updateRate"] = 1000.0;
    sensor_params[0]["visualize"] = false;
    sensor_params[0]["use_parallel"] = config.use_parallel;
    sensor_params[0]["resolution"] = config.resolution;
    sensor_params[0]["sampling_resolution"] = config.sampling_resolution;

    XmlRpc::XmlRpcValue surface_params;
    surface_params.setSize(1);
    surface_params[0]["type"] = "mujoco_contact_surfaces/MujocoContactSurfacesPlugin";
    surface_params[0]["SurfacePlugins"] = sensor_params;

    ros::param::set("MujocoPlugins", surface_params);
}

void run_config(Config &config, BenchResult &result, int bench_steps, int wait_steps) {
    std::string box_world = ros::package::getPath("mujoco_contact_surface_sensors") + "/assets/myrmex_box_world.xml";
    std::string plate_world = ros::package::getPath("mujoco_contact_surface_sensors") + "/assets/myrmex_plate_world.xml";
    std::string spot_world = ros::package::getPath("mujoco_contact_surface_sensors") + "/assets/myrmex_spot_world.xml";

    set_ros_config(config);

    config.print_config();

    std::string world;
    if (config.mesh == "Box") world = box_world;
    else if (config.mesh == "Plate") world = plate_world;
    else if (config.mesh == "Spot") world = spot_world;
    else {
        ROS_ERROR_STREAM("Unknown mesh type: " << config.mesh);
        return;
    }

    mujoco_ros::MujocoEnv env("");
    
    // Load box world
    mju::strcpy_arr(env.queued_filename_, world.c_str());
    env.settings_.load_request = 2;
    env.startPhysicsLoop();
    env.startEventLoop();

    float seconds = 0;
    while (env.getOperationalStatus() != 0 && seconds < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        seconds += 0.001;
    }

    if (seconds >= 2) {
        ROS_ERROR_STREAM("Env loading ran into 2 seconds timeout!");
        return;
    }

    // Get sensor plugin
    FlatTactileSensor *sensor_plugin;
    MujocoContactSurfacesPlugin *surfaces_plugin;

    auto plugins = env.getPlugins();
    for (auto p : plugins) {
        surfaces_plugin = dynamic_cast<MujocoContactSurfacesPlugin *>(p.get());
        if (surfaces_plugin == nullptr) {
            continue; // not the plugin we are looking for
        }
        for (auto sp_p : surfaces_plugin->getPlugins()) {
            sensor_plugin = dynamic_cast<FlatTactileSensor *>(sp_p.get());
            if (sensor_plugin != nullptr) {
                break;
            }
        }
    }

    if (sensor_plugin == nullptr) {
        ROS_ERROR_STREAM("Could not find flat sensor plugin!");
        return;
    }

    // without contact
    sensor_plugin->print_benchmark = false;
    surfaces_plugin->print_benchmark = false;

    env.step(bench_steps);

    Stats bvh_stat("BVH"), mt_stat("MT"), proj_stat("PROJ");
    sensor_plugin->benchmark_bvh.get_avgs(
        bvh_stat.bblas,
        bvh_stat.btlas,
        bvh_stat.nblas,
        bvh_stat.tr,
        bvh_stat.h,
        bvh_stat.total,
        bvh_stat.nrays,
        bvh_stat.ntri
    );

    sensor_plugin->benchmark_mt.get_avgs(
        mt_stat.bblas,
        mt_stat.btlas,
        mt_stat.nblas,
        mt_stat.tr,
        mt_stat.h,
        mt_stat.total,
        mt_stat.nrays,
        mt_stat.ntri
    );

    sensor_plugin->benchmark_proj.get_avgs(
        proj_stat.bblas,
        proj_stat.btlas,
        proj_stat.nblas,
        proj_stat.tr,
        proj_stat.h,
        proj_stat.total,
        proj_stat.nrays,
        proj_stat.ntri
    );

    Entry entry(config, bvh_stat, mt_stat, proj_stat);
    result.entries.push_back(entry);

    sensor_plugin->skip_update = true;
    env.step(wait_steps);
    sensor_plugin->skip_update = false;

    // Stabilize with contact
    env.step(bench_steps);

    sensor_plugin->benchmark_bvh.get_avgs(
        bvh_stat.bblas,
        bvh_stat.btlas,
        bvh_stat.nblas,
        bvh_stat.tr,
        bvh_stat.h,
        bvh_stat.total,
        bvh_stat.nrays,
        bvh_stat.ntri
    );

    sensor_plugin->benchmark_mt.get_avgs(
        mt_stat.bblas,
        mt_stat.btlas,
        mt_stat.nblas,
        mt_stat.tr,
        mt_stat.h,
        mt_stat.total,
        mt_stat.nrays,
        mt_stat.ntri
    );

    sensor_plugin->benchmark_proj.get_avgs(
        proj_stat.bblas,
        proj_stat.btlas,
        proj_stat.nblas,
        proj_stat.tr,
        proj_stat.h,
        proj_stat.total,
        proj_stat.nrays,
        proj_stat.ntri
    );

    entry = Entry(config, bvh_stat, mt_stat, proj_stat);
    result.entries.push_back(entry);

    env.settings_.exit_request = 1;
    env.waitForEventsJoin();
    env.waitForPhysicsJoin();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "flat_tactile_sensor_benchmark");

    BenchResult result;

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

    ros::NodeHandle nh("~");
    nh.setParam("unpause", false);
    nh.setParam("no_x", true);
    ros::param::set("use_sim_time", true);

    std::vector<Config> configs;
    std::string fname;

    // No parallel, scale sub_resolution
    configs.push_back(Config(false, 0.025, 4, "Box"));
    configs.push_back(Config(false, 0.025, 4,  "Plate"));
    configs.push_back(Config(false, 0.025, 4,  "Spot"));

    configs.push_back(Config(false, 0.025, 8, "Box"));
    configs.push_back(Config(false, 0.025, 8,  "Plate"));
    configs.push_back(Config(false, 0.025, 8,  "Spot"));

    configs.push_back(Config(false, 0.025, 16, "Box"));
    configs.push_back(Config(false, 0.025, 16, "Plate"));
    configs.push_back(Config(false, 0.025, 16, "Spot"));

    configs.push_back(Config(false, 0.025, 32, "Box"));
    configs.push_back(Config(false, 0.025, 32, "Plate"));
    configs.push_back(Config(false, 0.025, 32, "Spot"));

    for (Config config : configs) {
        run_config(config, result, 100, 300);
    }
    fname = "/tmp/bench_01.csv";
    ROS_INFO_STREAM("Benchmark for mesh sub_res finished! Dumping results to " << fname);
    result.to_csv(fname);

    configs.clear();

    // scale sub_resolution with parallel
    configs.push_back(Config(true, 0.025, 4, "Box"));
    configs.push_back(Config(true, 0.025, 4,  "Plate"));
    configs.push_back(Config(true, 0.025, 4,  "Spot"));

    configs.push_back(Config(true, 0.025, 8, "Box"));
    configs.push_back(Config(true, 0.025, 8,  "Plate"));
    configs.push_back(Config(true, 0.025, 8,  "Spot"));

    configs.push_back(Config(true, 0.025, 16, "Box"));
    configs.push_back(Config(true, 0.025, 16, "Plate"));
    configs.push_back(Config(true, 0.025, 16, "Spot"));

    configs.push_back(Config(true, 0.025, 32, "Box"));
    configs.push_back(Config(true, 0.025, 32, "Plate"));
    configs.push_back(Config(true, 0.025, 32, "Spot"));

    for (Config config : configs) {
        run_config(config, result, 100, 300);
    }
    fname = "/tmp/bench_02.csv";
    ROS_INFO_STREAM("Benchmark for sub_res parallel finished! Dumping results to " << fname);
    result.to_csv(fname);

    configs.clear();

    // No parallel scale resolution and sub_resolution
    configs.push_back(Config(false, 0.0025, 4, "Box"));
    configs.push_back(Config(false, 0.0025, 8, "Box"));
    configs.push_back(Config(false, 0.0025, 16, "Box"));
    configs.push_back(Config(false, 0.0025, 32, "Box"));

    configs.push_back(Config(false, 0.0025, 4,  "Plate"));
    configs.push_back(Config(false, 0.0025, 8,  "Plate"));
    configs.push_back(Config(false, 0.0025, 16, "Plate"));
    configs.push_back(Config(false, 0.0025, 32, "Plate"));

    configs.push_back(Config(false, 0.0025, 4,  "Spot"));
    configs.push_back(Config(false, 0.0025, 8,  "Spot"));
    configs.push_back(Config(false, 0.0025, 16, "Spot"));
    configs.push_back(Config(false, 0.0025, 32, "Spot"));

    for (Config config : configs) {
        run_config(config, result, 100, 300);
    }
    fname = "/tmp/bench_03.csv";
    ROS_INFO_STREAM("Benchmark for scale res and sub_res finished! Dumping results to " << fname);
    result.to_csv(fname);

    configs.clear();

    // scale resolution and sub_resolution with parallel
    configs.push_back(Config(true, 0.0025, 4, "Box"));
    configs.push_back(Config(true, 0.0025, 4,  "Plate"));
    configs.push_back(Config(true, 0.0025, 4,  "Spot"));

    configs.push_back(Config(true, 0.0025, 8, "Box"));
    configs.push_back(Config(true, 0.0025, 8,  "Plate"));
    configs.push_back(Config(true, 0.0025, 8,  "Spot"));

    configs.push_back(Config(true, 0.0025, 16, "Box"));
    configs.push_back(Config(true, 0.0025, 16, "Plate"));
    configs.push_back(Config(true, 0.0025, 16, "Spot"));

    configs.push_back(Config(true, 0.0025, 32, "Box"));
    configs.push_back(Config(true, 0.0025, 32, "Plate"));
    configs.push_back(Config(true, 0.0025, 32, "Spot"));

    for (Config config : configs) {
        run_config(config, result, 100, 300);
    }
    fname = "/tmp/bench_04.csv";
    ROS_INFO_STREAM("Benchmark finished! Dumping results to " << fname);
    result.to_csv(fname);

    return 0;
}