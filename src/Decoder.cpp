#include "rosneuro_decoder/Decoder.h"

namespace rosneuro{
    namespace decoder{
        Decoder::Decoder(void) : p_nh_("~"){
            this->has_new_data_ = false;
            this->is_first_message_ = true;

            this->loader_.reset(new pluginlib::ClassLoader<GenericDecoder>("rosneuro_decoder", "rosneuro::decoder::GenericDecoder"));
        }

        Decoder::~Decoder(void){
            boost::shared_ptr<GenericDecoder>().swap(this->decoder_);
            this->loader_.reset();
        }

        std::string Decoder::path(){
            if(!this->decoder_->isSet()){
                ROS_ERROR("[Decoder] not configured yet");
                throw std::runtime_error("[Decoder] not configured yet");
            }

            return this->decoder_->path();
        }

        std::string Decoder::name(){
            if(!this->decoder_->isSet()){
                ROS_ERROR("[Decoder] not configured yet");
                throw std::runtime_error("[Decoder] not configured yet");
            }

            return this->decoder_->name();
        }

        std::vector<int> Decoder::classes(){
            if(!this->decoder_->isSet()){
                ROS_ERROR("[Decoder] not configured yet");
                throw std::runtime_error("[Decoder] not configured yet");
            }

            return this->decoder_->classes();
        }

        bool Decoder::configure(void){
            // get the plugin of the decoder
            if(ros::param::get("~plugin", this->plugin_) == false){
                ROS_ERROR("[decoder] Missing 'plugin' parameter, which is a mandatory parameter");
                return false;
            }
            
            // Dynamically load the plugin and create the decoder class istance
            try{ 
                this->decoder_ = this->loader_->createInstance(this->plugin_);
            } catch(pluginlib::PluginlibException& ex){
                ROS_ERROR("[decoder] '%s' decoder failed to create: %s", this->plugin_.c_str(), ex.what());
                return false;
            }

            this->decodername_ = this->decoder_->name();

            // take the name of cfg decoder configuration
            std::string cfg_name;
            if(ros::param::get("~cfg_name", cfg_name) == false){
                ROS_ERROR("[%s] Missing 'cfg_name' parameter, which is a mandatory parameter", this->decodername_.c_str());
                return false;
            }

            // Configure the decoder
            if(this->decoder_->configure(cfg_name) == false){
                ROS_ERROR("[%s] Cannot configure the decoder", this->decodername_.c_str());
                return false;
            }

            // Subscribers and publisher
            //this->sub_ = this->nh_.subscribe("/smr/features",1, &Decoder::on_received_data, this);
            this->pub_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>("/smr/neuroprediction", 1);

            ROS_INFO("[%s] Decoder correctly created and configured", this->decodername_.c_str());
            return true;
        }

        void Decoder::run(void){
            ros::Rate r(512);

            while(ros::ok()){
                if(this->has_new_data_ == true){
                    this->set_message(this->output_);
                    this->pub_.publish(this->msgoutput_);
                    this->has_new_data_ = false;
                }

                ros::spinOnce();
                r.sleep();
            }

        }

        Eigen::VectorXf Decoder::apply(const Eigen::VectorXf& in){
            Eigen::VectorXf out = this->decoder_->apply(in);
            return out;

        }

        Eigen::VectorXf Decoder::getFeatures(const Eigen::MatrixXf& in){
            Eigen::VectorXf features = this->decoder_->getFeatures(in);
            return features;
        }

        /*  TODO: define message from pwelch and read it
        void Decoder::on_received_data(const rosneuro_msgs::NeuroOutput& msg){
            this->input_ = this->vector_to_eigen(msg.softpredict.data);
            this-> output_ = this->decoder_->apply(this->input_);
            this->has_new_data_ = true;

            this->msgoutput_.neuroheader = msg.neuroheader;

            if(this->is_first_message_ == true){
                this->msgoutput_.hardpredict.data = msg.hardpredict.data;
                this->msgoutput_.decoder.classes = msg.decoder.classes;
                this->msgoutput_.decoder.type = msg.decoder.type;
                this->msgoutput_.decoder.path = msg.decoder.path;
                this->is_first_message_ = false;
            }
        }
        */

        void Decoder::set_message(const Eigen::VectorXf& data){
            this->msgoutput_.header.stamp = ros::Time::now();
            this->msgoutput_.softpredict.data = this-> eigen_to_vector(data);
        }

        Eigen::VectorXf Decoder::vector_to_eigen(const std::vector<float>& in) {

	        float* ptr_in;

        	ptr_in = const_cast<float*>(in.data());

        	Eigen::VectorXf out = Eigen::Map<Eigen::VectorXf>(ptr_in, in.size());

        	return out;
        }

        std::vector<float> Decoder::eigen_to_vector(const Eigen::VectorXf& in) {

        	std::vector<float> out(in.size());

        	Eigen::Map<Eigen::VectorXf>(out.data(), in.size()) = in;

        	return out;

        }

    }
}