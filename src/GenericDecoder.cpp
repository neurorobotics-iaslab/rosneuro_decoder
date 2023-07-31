#include "rosneuro_decoder/GenericDecoder.h"

namespace rosneuro {
	namespace decoder {

		GenericDecoder::GenericDecoder(void) {
			this->name_ = "undefined";
			this->is_configured_ = false;
		}

		GenericDecoder::~GenericDecoder(void) {}

		std::string GenericDecoder::name(void) {
			return this->name_;
		}

		void GenericDecoder::setname(const std::string& name) {
			this->name_ = name;
		}

		bool GenericDecoder::isSet(void){
			return this->is_configured_;
		}

		bool GenericDecoder::configure(const std::string& param_name) {
			
			bool retval = false;
			XmlRpc::XmlRpcValue config;
			if (!this->nh_.getParam(param_name, config)) {
		  		ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
				return false;
			}
		
			retval = this->configure(config);
			return retval;
		}
		
		bool GenericDecoder::configure(XmlRpc::XmlRpcValue& config) {
			if (this->is_configured_) {
				ROS_ERROR("Decoder %s already being reconfigured", this->name_.c_str());
			}
			this->is_configured_ = false;
			bool retval = true;
			
			retval = retval && this->loadConfiguration(config);
			retval = retval && this->configure();
			this->is_configured_ = retval;
			return retval;
		}
		
		bool GenericDecoder::loadConfiguration(XmlRpc::XmlRpcValue& config) {
			
			if(config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
				ROS_ERROR("A Decoder configuration must be a map with fields name, type, and params");
				return false;
			} 
		
			if (!setNameAndType(config)) {
				return false;
			}
		
			//check to see if we have parameters in our list
			if(config.hasMember("params")) {
			
				//get the params map
		  		XmlRpc::XmlRpcValue params = config["params"];
		
				if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
		    		ROS_ERROR("params must be a map");
					return false;
				} else {
				
					//Load params into map
		    		for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
		      			ROS_DEBUG("Loading param %s", it->first.c_str());
		      			this->params_[it->first] = it->second;
					}
				}
			}


			return true;
		}

		bool GenericDecoder::setNameAndType(XmlRpc::XmlRpcValue& config) {

			if(config.hasMember("name") == false) {
				ROS_ERROR("Decoder didn't have name defined, this is required");
				return false;
			}

			this->name_ = std::string(config["name"]);
			ROS_DEBUG("Configuring Decoder with name %s", this->name().c_str());

			return true;
		}

		bool GenericDecoder::getParam(const std::string& name, std::string& value) const {
    
			auto it = this->params_.find(name);
			if (it == this->params_.end())
				return false;

		    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeString)
				return false;

		    auto tmp = it->second;
		    value = std::string(tmp);
		    return true;
		}

		bool GenericDecoder::getParam(const std::string& name, int& value) const {
			auto it = this->params_.find(name);
			if (it == this->params_.end())
				return false;

			if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
				return false;

			auto tmp = it->second;
			value = tmp;
			return true;
		}

		bool GenericDecoder::getParam(const std::string& name, double& value) const {
			auto it = this->params_.find(name);
			if (it == this->params_.end())
				return false;

			if(it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble)
				return false;

			auto tmp = it->second;
			value = (double) tmp;
			return true;
		}

		bool GenericDecoder::getParam(const std::string& name, unsigned int& value) const {
			int signed_value;
			if (!this->getParam(name, signed_value))
				return false;
			if (signed_value < 0)
				return false;
			value = signed_value;
			return true;
		}

		bool GenericDecoder::getParam(const std::string& name, std::vector<uint32_t>& value) const {
			auto it = this->params_.find(name);
			if (it == this->params_.end())
				return false;

			value.clear();

			if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
				return false;

			XmlRpc::XmlRpcValue double_array = it->second;

			for (auto i = 0; i < double_array.size(); ++i){
				if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
			    return false;
				}

				uint32_t double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (int)(double_array[i]);
				value.push_back(double_value);
			}

			return true;
		}

		bool GenericDecoder::getParam(const std::string& name, std::vector<double>& value) const {
			auto it = this->params_.find(name);
			if (it == this->params_.end())
				return false;

			value.clear();

			if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
				return false;

			XmlRpc::XmlRpcValue double_array = it->second;

			for (auto i = 0; i < double_array.size(); ++i){
				if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {

			    return false;
				}

				double double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (double) (double_array[i]) : (double)(double_array[i]);
				value.push_back(double_value);
			}

			return true;
		}

		// load vector of vector
        bool GenericDecoder::load_vectorOfVector(const std::string current_str, std::vector<std::vector<uint32_t>>& out){
            unsigned int nrows;
            unsigned int ncols;

            std::stringstream ss(current_str);
            std::string c_row;

            while(getline(ss, c_row, ';')){
                std::stringstream iss(c_row);
                int index;
                std::vector<uint32_t> row;
                while(iss >> index){
                    row.push_back(index);
                }
                out.push_back(row);
            }

            return true;
        }

		// load eigen matrix
        bool GenericDecoder::load_eigen(const std::string current_str, Eigen::Ref<Eigen::MatrixXf> out){
            unsigned int nrows;
            unsigned int ncols;
            std::vector<std::vector<float>> temp_matrix;

            std::stringstream ss(current_str);
            std::string c_row;

            while(getline(ss, c_row, ';')){
                std::stringstream iss(c_row);
                float index;
                std::vector<float> row;
                while(iss >> index){
                    row.push_back(index);
                }
                temp_matrix.push_back(row);
            }

            nrows = temp_matrix.size();
            ncols = temp_matrix.at(0).size();

            // check if correct dimension with the provided data config
            if(nrows != out.rows() || ncols != out.cols()){
                return false;
            }

            // check if always same dimension in temp_matrix
            for(auto it=temp_matrix.begin(); it != temp_matrix.end(); ++it){
                if((*it).size() != ncols){
                    return false;
                }
            }

            out = Eigen::MatrixXf::Zero(nrows, ncols);

            for(auto i = 0; i < out.rows(); i++){
                for(auto j = 0; j < out.cols(); j++){
                    out(i,j) = temp_matrix.at(i).at(j);
                }
            }

            return true;
        }

	}

}
