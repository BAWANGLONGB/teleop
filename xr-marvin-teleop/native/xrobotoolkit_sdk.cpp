#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <json-c/json.h>

#include <PXREARobotSDK.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace py = pybind11;

namespace
{

using Pose = std::array<double, 7>;
using JsonObject = std::unique_ptr<json_object, decltype(&json_object_put)>;

struct XrSnapshot
{
    int64_t timestamp_ns;
    Pose headset_pose;
    Pose left_controller_pose;
    Pose right_controller_pose;
    std::array<double, 2> grip_values;
    bool button_a;
    bool button_b;
};

std::mutex snapshot_mutex;
std::optional<XrSnapshot> latest_snapshot;
std::mutex lifecycle_mutex;
bool is_initialized = false;
std::atomic<bool> parse_error_logged = false;

json_object* require_member(json_object* object, const char* name)
{
    json_object* value = nullptr;
    if (object == nullptr || !json_object_object_get_ex(object, name, &value) ||
        value == nullptr)
    {
        throw std::runtime_error(std::string("missing XR field: ") + name);
    }
    return value;
}

Pose parse_pose(json_object* object)
{
    const char* encoded_pose = json_object_get_string(require_member(object, "pose"));
    if (encoded_pose == nullptr)
    {
        throw std::runtime_error("XR pose is not a string");
    }

    Pose pose{};
    std::stringstream stream(encoded_pose);
    std::string value;
    for (double& coordinate : pose)
    {
        if (!std::getline(stream, value, ','))
        {
            throw std::runtime_error("XR pose does not contain seven values");
        }
        coordinate = std::stod(value);
    }
    if (std::getline(stream, value, ','))
    {
        throw std::runtime_error("XR pose contains more than seven values");
    }
    return pose;
}

JsonObject parse_json(const char* encoded_json)
{
    if (encoded_json == nullptr)
    {
        throw std::runtime_error("XR JSON is null");
    }
    JsonObject object(json_tokener_parse(encoded_json), &json_object_put);
    if (!object)
    {
        throw std::runtime_error("invalid XR JSON");
    }
    return object;
}

XrSnapshot parse_snapshot(const PXREADevStateJson& device_state)
{
    JsonObject envelope = parse_json(device_state.stateJson);
    const char* encoded_value =
        json_object_get_string(require_member(envelope.get(), "value"));
    JsonObject value = parse_json(encoded_value);

    json_object* controllers = require_member(value.get(), "Controller");
    json_object* left_controller = require_member(controllers, "left");
    json_object* right_controller = require_member(controllers, "right");
    json_object* headset = require_member(value.get(), "Head");

    XrSnapshot snapshot{};
    snapshot.timestamp_ns =
        json_object_get_int64(require_member(value.get(), "timeStampNs"));
    snapshot.headset_pose = parse_pose(headset);
    snapshot.left_controller_pose = parse_pose(left_controller);
    snapshot.right_controller_pose = parse_pose(right_controller);
    snapshot.grip_values = {
        json_object_get_double(require_member(left_controller, "grip")),
        json_object_get_double(require_member(right_controller, "grip")),
    };
    snapshot.button_a =
        json_object_get_boolean(require_member(right_controller, "primaryButton"));
    snapshot.button_b =
        json_object_get_boolean(require_member(right_controller, "secondaryButton"));
    if (snapshot.timestamp_ns <= 0)
    {
        throw std::runtime_error("XR timestamp is not positive");
    }
    return snapshot;
}

void on_client_callback(void*, PXREAClientCallbackType type, int, void* user_data)
{
    if (type != PXREADeviceStateJson || user_data == nullptr)
    {
        return;
    }

    try
    {
        XrSnapshot snapshot =
            parse_snapshot(*static_cast<PXREADevStateJson*>(user_data));
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        latest_snapshot = std::move(snapshot);
    }
    catch (const std::exception& error)
    {
        if (!parse_error_logged.exchange(true))
        {
            std::cerr << "XRoboToolkit snapshot rejected: " << error.what() << std::endl;
        }
    }
}

void initialize()
{
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex);
    if (is_initialized)
    {
        return;
    }
    {
        std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex);
        latest_snapshot.reset();
    }
    parse_error_logged = false;
    if (PXREAInit(nullptr, on_client_callback, PXREAFullMask) != 0)
    {
        throw std::runtime_error("PXREAInit failed");
    }
    is_initialized = true;
}

void close_sdk()
{
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex);
    if (!is_initialized)
    {
        return;
    }
    PXREADeinit();
    is_initialized = false;
    std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex);
    latest_snapshot.reset();
}

py::object get_snapshot()
{
    std::optional<XrSnapshot> snapshot;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        snapshot = latest_snapshot;
    }
    if (!snapshot)
    {
        return py::none();
    }

    py::dict result;
    result["timestamp_ns"] = snapshot->timestamp_ns;
    result["headset_pose"] = snapshot->headset_pose;
    result["left_controller_pose"] = snapshot->left_controller_pose;
    result["right_controller_pose"] = snapshot->right_controller_pose;
    result["grip_values"] = snapshot->grip_values;
    result["button_a"] = snapshot->button_a;
    result["button_b"] = snapshot->button_b;
    return result;
}

} // namespace

PYBIND11_MODULE(_xrobotoolkit_sdk, module)
{
    module.def("init", &initialize, "Initialize XRoboToolkit reception.");
    module.def("close", &close_sdk, "Stop XRoboToolkit reception.");
    module.def("get_snapshot", &get_snapshot, "Copy the latest complete PICO XR frame.");
}
