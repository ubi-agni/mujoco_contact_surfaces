/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Florian Patzelt*/

#include <mujoco_contact_surfaces/plugin_utils.h>

namespace mujoco_ros::contact_surfaces::plugin_utils {

bool parsePlugins(const XmlRpc::XmlRpcValue &config,
                  boost::shared_ptr<pluginlib::ClassLoader<SurfacePlugin>> &plugin_loader_ptr_,
                  XmlRpc::XmlRpcValue &plugin_config_rpc)
{
	if (config.hasMember(SURFACE_PLUGIN_PARAM_NAME)) {
		ROS_DEBUG_STREAM_NAMED("surface_plugin_loader", "Found SurfacePlugins param under " << SURFACE_PLUGIN_PARAM_NAME);
	} else {
		ROS_INFO_NAMED("surface_plugin_loader", "No plugins to load listed in parameter server!");
		return false;
	}

	plugin_config_rpc = config[SURFACE_PLUGIN_PARAM_NAME];

	if (plugin_config_rpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		ROS_ERROR_STREAM_NAMED("mujoco_contact_surfaces_plugin_loader",
		                       "Error while parsing SurfacePlugins rosparam: wrong type.");
		ROS_DEBUG_STREAM_NAMED("mujoco_contact_surfaces_plugin_loader",
		                       "SurfacePlugins rosparam should be of type '"
		                           << XmlRpc::XmlRpcValue::TypeArray << "', but got type '" << plugin_config_rpc.getType()
		                           << "' (yaml array)!");
		return false;
	} else {
		ROS_DEBUG_NAMED("mujoco_contact_surfaces_plugin_loader", "Initializing plugin loader ... ");
		plugin_loader_ptr_.reset(new pluginlib::ClassLoader<SurfacePlugin>(
		    "mujoco_contact_surfaces", "mujoco_ros::contact_surfaces::SurfacePlugin"));
	}
	return true;
}

void registerPlugins(const std::string &nh_namespace, const XmlRpc::XmlRpcValue &config_rpc,
                     boost::shared_ptr<pluginlib::ClassLoader<SurfacePlugin>> &plugin_loader_ptr_,
                     std::vector<SurfacePluginPtr> &plugins)
{
	for (uint i = 0; i < config_rpc.size(); i++) {
		if (config_rpc[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			ROS_ERROR_STREAM_NAMED("mujoco_contact_surfaces_plugin_loader",
			                       "Error while parsing SurfacePlugins rosparam: wrong type.");
			ROS_DEBUG_STREAM_NAMED("mujoco_contact_surfaces_plugin_loader",
			                       "Children of 'SurfacePlugins' should be of type '"
			                           << XmlRpc::XmlRpcValue::TypeStruct << "', but got type '" << config_rpc.getType()
			                           << "'. Skipping " << config_rpc[i]);
			continue;
		}
		registerPlugin(nh_namespace, config_rpc[i], plugin_loader_ptr_, plugins);
	}
}

bool registerPlugin(const std::string &nh_namespace, const XmlRpc::XmlRpcValue &config,
                    boost::shared_ptr<pluginlib::ClassLoader<SurfacePlugin>> &plugin_loader_ptr_,
                    std::vector<SurfacePluginPtr> &plugins)
{
	ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	std::string type;

	if (!config.hasMember("type")) {
		ROS_ERROR_NAMED("mujoco_contact_surfaces_plugin_loader",
		                "Error while parsing SurfacePlugins rosparam: Every listed plugin "
		                "should provide a 'type' member!");
		return false;
	}
	type = (std::string)config["type"];

	ROS_DEBUG_STREAM_NAMED("mujoco_contact_surfaces_plugin_loader", "Registering plugin of type " << type);

	try {
		SurfacePluginPtr mjplugin_ptr = plugin_loader_ptr_->createInstance(type);
		mjplugin_ptr->init(config, nh_namespace);
		plugins.push_back(mjplugin_ptr);
		ROS_DEBUG_STREAM_NAMED("mujoco_contact_surfaces_plugin_loader",
		                       "Added " << type << " to the list of loaded plugins in namespace '" << nh_namespace
		                                << "'. List now contains " << plugins.size() << " plugin(s)");
	} catch (const pluginlib::PluginlibException &ex) {
		ROS_ERROR_STREAM_NAMED("mujoco_contact_surfaces_plugin_loader",
		                       "The plugin failed to load (for namespace " << nh_namespace << " ): " << ex.what());
		return false;
	}

	return true;
}

} // namespace mujoco_ros::contact_surfaces::plugin_utils
