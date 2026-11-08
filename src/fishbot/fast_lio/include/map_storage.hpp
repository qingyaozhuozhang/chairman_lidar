#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>
#include <pcl/io/pcd_io.h>

namespace fast_lio_storage {
namespace fs = std::filesystem;

inline fs::path package_root(const std::string &workspace)
{
    const fs::path relative = "src/fishbot/fast_lio";
    if (!workspace.empty()) {
        const auto source = fs::absolute(workspace) / relative;
        if (fs::is_regular_file(source / "package.xml")) {
            return fs::weakly_canonical(source);
        }
        throw std::runtime_error("workspace_root does not contain src/fishbot/fast_lio/package.xml");
    }
    std::error_code error;
    std::vector<fs::path> starts;
    const auto executable = fs::read_symlink("/proc/self/exe", error);
    if (!error) starts.push_back(executable.parent_path());
    starts.push_back(fs::current_path());
    for (auto candidate : starts) {
        while (!candidate.empty()) {
            const auto source = candidate / relative;
            if (fs::is_regular_file(source / "package.xml")) {
                return fs::weakly_canonical(source);
            }
            const auto parent = candidate.parent_path();
            if (parent == candidate) break;
            candidate = parent;
        }
    }
    throw std::runtime_error("Cannot find FAST-LIO source package; set the workspace_root parameter");
}

template<typename PointT>
void save_atomic(const fs::path &path, const pcl::PointCloud<PointT> &cloud)
{
    if (cloud.empty()) throw std::runtime_error("No points collected; map was not saved");
    fs::create_directories(path.parent_path());
    const auto temporary = path.parent_path() /
        ("." + path.filename().string() + "." + std::to_string(getpid()) + ".tmp");
    try {
        pcl::PCDWriter writer;
        if (writer.writeBinary(temporary.string(), cloud) < 0) {
            throw std::runtime_error("PCD write failed: " + path.string());
        }
        fs::rename(temporary, path);
    } catch (...) {
        std::error_code ignored;
        fs::remove(temporary, ignored);
        throw;
    }
}
}  // namespace fast_lio_storage
