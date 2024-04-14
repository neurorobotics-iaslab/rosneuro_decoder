#include "rosneuro_decoder/GenericDecoder.h"

namespace rosneuro {
	namespace decoder {

		GenericDecoder::GenericDecoder(void) {
			this->name_ = "undefined";
			this->is_configured_ = false;
		}

		GenericDecoder::~GenericDecoder(void) {}

		std::string GenericDecoder::getName(void) {
			return this->name_;
		}

		void GenericDecoder::setName(const std::string& name) {
			this->name_ = name;
		}

		bool GenericDecoder::isSet(void){
			return this->is_configured_;
		}

		bool GenericDecoder::configure(const std::string& param_name) {
			XmlRpc::XmlRpcValue config;
			if (!this->nh_.getParam(param_name, config)) {
		  		ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
				return false;
			}
            return this->configure(config);
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
		
			if(config.hasMember("params")) {
		  		XmlRpc::XmlRpcValue params = config["params"];
				if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
		    		ROS_ERROR("params must be a map");
					return false;
				} else {
		    		for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
		      			ROS_DEBUG("Loading param %s", it->first.c_str());
		      			this->params_[it->first] = it->second;
					}
				}
			}
			return true;
		}

		bool GenericDecoder::setNameAndType(XmlRpc::XmlRpcValue& config) {
			if(!config.hasMember("name")) {
				ROS_ERROR("Decoder didn't have name defined, this is required");
				return false;
			}
			this->name_ = std::string(config["name"]);
			ROS_DEBUG("Configuring Decoder with name %s", this->getName().c_str());

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

			auto to_convert = it->second;
			value = (double) to_convert;
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

        bool GenericDecoder::loadVectorOfVector(const std::string current_str, std::vector<std::vector<uint32_t>>& out){
            return parseStringToMatrix(current_str, out);
        }

        template<typename T>
        bool GenericDecoder::parseStringToMatrix(const std::string current_str, std::vector<std::vector<T>>& out){
            std::stringstream ss(current_str);
            std::string c_row;

            while(getline(ss, c_row, ';')){
                std::stringstream iss(c_row);
                T index;
                std::vector<T> row;
                while(iss >> index){
                    row.push_back(index);
                }
                out.push_back(row);
            }
            return true;
        }

        bool GenericDecoder::loadEigen(const std::string current_str, Eigen::Ref<Eigen::MatrixXf> out){
            std::vector<std::vector<float>> matrix;

            if (!parseStringToMatrix(current_str, matrix)) {
                return false;
            }

            if (!validateMatrixDimensions(matrix, out)) {
                return false;
            }

            unsigned int n_rows = matrix.size();
            unsigned int n_cols = matrix.at(0).size();
            out = Eigen::MatrixXf::Zero(n_rows, n_cols);
            populateEigenMatrix(matrix, out);

            return true;
        }

        bool GenericDecoder::validateMatrixDimensions(const std::vector<std::vector<float>>& matrix, Eigen::Ref<Eigen::MatrixXf> out) {
            unsigned int n_rows = matrix.size();
            unsigned int n_cols = matrix.at(0).size();

            if(n_rows != out.rows() || n_cols != out.cols()){
                return false;
            }

            for(auto it=matrix.begin(); it != matrix.end(); ++it){
                if((*it).size() != n_cols){
                    return false;
                }
            }

            return true;
        }

        void GenericDecoder::populateEigenMatrix(const std::vector<std::vector<float>>& input, Eigen::Ref<Eigen::MatrixXf> output) {
            for (size_t i = 0; i < input.size(); ++i) {
                for (size_t j = 0; j < input[i].size(); ++j) {
                    output(i, j) = input[i][j];
                }
            }
        }
	}

}
