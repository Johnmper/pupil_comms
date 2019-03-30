/**
@author     johnmper
@file       pupil_adaptor.hh
@brief      Adaptors implemented for the msgpack deserialization!
*/
#ifndef PUPIL_ADAPTOR_HEADER
#define PUPIL_ADAPTOR_HEADER

#include<iostream>
#include<string>
#include<chrono>
#include<functional>

#include<pupil_msgs/point2d.h>
#include<pupil_msgs/point3d.h>
#include<pupil_msgs/ellipse2d.h>
#include<pupil_msgs/circle3d.h>
#include<pupil_msgs/sphere3d.h>
#include<pupil_msgs/gaze_datum.h>
#include<pupil_msgs/pupil_datum.h>
#include<pupil_msgs/frame.h>

#include<msgpack.hpp>

// Define DEBUG_ALL to show all the information
#undef DEBUG_ALL

#ifdef DEBUG_ALL
#define ADAPTOR_DEBUG
#define TIME_DEBUG
#endif

std::chrono::high_resolution_clock::time_point t1,t2;
double debug_duration=0.0;

#ifdef TIME_DEBUG
// Usage:
//  TIME_FUNCTION_DEBUG( fun() )
// Output:
//  $ Took: xxx us
#define TIME_FUNCTION_DEBUG(a) \
            t1 = std::chrono::high_resolution_clock::now();\
            a;\
            t2 = std::chrono::high_resolution_clock::now();\
            debug_duration = (double)std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();\
            std::cout << "Took: " << debug_duration << " us" << std::endl;

#else
#define TIME_FUNCTION_DEBUG(a) a;
#endif

#ifdef ADAPTOR_DEBUG
#define MAP_HASH_DEBUG(a) \
            std::cout << "==== HASH RESULTS : " << a << std::endl;\
            for(unsigned int kk=0; kk < o.via.map.size; ++kk){\
                std::string key_str = o.via.map.ptr[kk].key.as<std::string>();\
                std::stringstream ss,ss2,ss3;\
                switch(o.via.map.ptr[kk].val.type){\
                    case msgpack::type::NIL:\
                        ss << std::setw(10) << std::right << "[NIL] "; break;\
                    case msgpack::type::BOOLEAN:\
                        ss << std::setw(10) << std::right << "[BOOL] "; break;\
                    case msgpack::type::POSITIVE_INTEGER:\
                        ss << std::setw(10) << std::right << "[+INT] "; break;\
                    case msgpack::type::NEGATIVE_INTEGER:\
                        ss << std::setw(10) << std::right << "[-INT] "; break;\
                    case msgpack::type::FLOAT32:\
                        ss << std::setw(10) << std::right << "[FLOAT32] "; break;\
                    case msgpack::type::FLOAT64:\
                        ss << std::setw(10) << std::right << "[FLOAT64] "; break;\
                    case msgpack::type::STR:\
                        ss << std::setw(10) << std::right << "[STRING] "; break;\
                    case msgpack::type::ARRAY:\
                        ss << std::setw(10) << std::right << "[ARRAY] "; break;\
                    case msgpack::type::MAP:\
                        ss << std::setw(10) << std::right << "[MAP] "; break;\
                    case msgpack::type::EXT:\
                        ss << std::setw(10) << std::right << "[EXT] "; break;\
                    case msgpack::type::BIN:\
                        ss << std::setw(10) << std::right << "[BIN] "; break;\
                }\
                ss2 << "(" <<  std::hash<std::string>{}(key_str) <<")";\
                std::cout << ss.str() << std::setw(6) << std::right << ss2.str() << " " << key_str << " -> " <<  o.via.map.ptr[kk].val <<std::endl;\
            }\
            std::cout << "==============================" << std::endl<<std::endl << std::endl;
#else
#define MAP_HASH_DEBUG(a)
#endif


// Better readability with ros msgs
typedef float float32_t;
typedef double float64_t;


namespace msgpack {
MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
namespace adaptor {

//===========================================================================================================
//     ########   #######  #### ##    ## ########  #######  ########
//     ##     ## ##     ##  ##  ###   ##    ##    ##     ## ##     ##
//     ##     ## ##     ##  ##  ####  ##    ##           ## ##     ##
//     ########  ##     ##  ##  ## ## ##    ##     #######  ##     ##
//     ##        ##     ##  ##  ##  ####    ##    ##        ##     ##
//     ##        ##     ##  ##  ##   ###    ##    ##        ##     ##
//     ##         #######  #### ##    ##    ##    ######### ########
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::point2d> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::point2d& msg) const{
            if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
            if (o.via.array.size != 2) throw msgpack::type_error();

            msg.x = o.via.array.ptr[0].as<float64_t>();
            msg.y = o.via.array.ptr[1].as<float64_t>();

            return o;
        }
    };


//===========================================================================================================
//     ########   #######  #### ##    ## ########  #######  ########
//     ##     ## ##     ##  ##  ###   ##    ##    ##     ## ##     ##
//     ##     ## ##     ##  ##  ####  ##    ##           ## ##     ##
//     ########  ##     ##  ##  ## ## ##    ##     #######  ##     ##
//     ##        ##     ##  ##  ##  ####    ##           ## ##     ##
//     ##        ##     ##  ##  ##   ###    ##    ##     ## ##     ##
//     ##         #######  #### ##    ##    ##     #######  ########
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::point3d> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::point3d& msg) const{
            if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
            if (o.via.array.size != 3) throw msgpack::type_error();

            msg.x = o.via.array.ptr[0].as<float64_t>();
            msg.y = o.via.array.ptr[1].as<float64_t>();
            msg.z = o.via.array.ptr[2].as<float64_t>();

            return o;
        }
    };


//===========================================================================================================
//     ######## ##       ##       #### ########   ######  ########  #######  ########
//     ##       ##       ##        ##  ##     ## ##    ## ##       ##     ## ##     ##
//     ##       ##       ##        ##  ##     ## ##       ##              ## ##     ##
//     ######   ##       ##        ##  ########   ######  ######    #######  ##     ##
//     ##       ##       ##        ##  ##              ## ##       ##        ##     ##
//     ##       ##       ##        ##  ##        ##    ## ##       ##        ##     ##
//     ######## ######## ######## #### ##         ######  ######## ######### ########
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::ellipse2d> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::ellipse2d& msg) const{
            if (o.type != msgpack::type::MAP) throw msgpack::type_error();
            if (o.via.map.size != 3) throw msgpack::type_error();
            MAP_HASH_DEBUG("ELLIPSE2D");

            for(unsigned int ii=0; ii < o.via.map.size; ++ii){
                std::string key_str = o.via.map.ptr[ii].key.as<std::string>();
                // Const reference to val object, simplifies the code, adds to the readability
                const msgpack::object& val = o.via.map.ptr[ii].val;
                switch(std::hash<std::string>{}(key_str)){
                    case 14711809123715985559U:   // float64 angle
                        msg.angle = val.as<float64_t>(); break;
                    case 12974207293192780750U:   // point2d axes
                        val.convert<pupil_msgs::point2d>(msg.axes); break;
                    case 10907956872829995589U:   // point2d center
                        val.convert<pupil_msgs::point2d>(msg.center); break;
                }
            }

            return o;
        }
    };

//===========================================================================================================
//      ######  #### ########   ######  ##       ########  #######  ########
//     ##    ##  ##  ##     ## ##    ## ##       ##       ##     ## ##     ##
//     ##        ##  ##     ## ##       ##       ##              ## ##     ##
//     ##        ##  ########  ##       ##       ######    #######  ##     ##
//     ##        ##  ##   ##   ##       ##       ##              ## ##     ##
//     ##    ##  ##  ##    ##  ##    ## ##       ##       ##     ## ##     ##
//      ######  #### ##     ##  ######  ######## ########  #######  ########
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::circle3d> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::circle3d& msg) const{
            if (o.type != msgpack::type::MAP) throw msgpack::type_error();
            if (o.via.map.size != 3) throw msgpack::type_error();
            MAP_HASH_DEBUG("CIRCLE3D");

            for(unsigned int ii=0; ii < o.via.map.size; ++ii){
                std::string key_str = o.via.map.ptr[ii].key.as<std::string>();
                // Const reference to val object, simplifies the code, adds to the readability
                const msgpack::object& val = o.via.map.ptr[ii].val;
                switch(std::hash<std::string>{}(key_str)){
                    case 17715334705756511795U:   // float64 radius
                        msg.radius = val.as<float64_t>(); break;
                    case 10907956872829995589U:   // point3d center
                        val.convert<pupil_msgs::point3d>(msg.center); break;
                    case 5915016993924152602U:   // point3d normal
                        val.convert<pupil_msgs::point3d>(msg.normal); break;
                }
            }

            return o;
        }
    };


//===========================================================================================================
//      ######  ########  ##     ## ######## ########  ########  #######  ########
//     ##    ## ##     ## ##     ## ##       ##     ## ##       ##     ## ##     ##
//     ##       ##     ## ##     ## ##       ##     ## ##              ## ##     ##
//      ######  ########  ######### ######   ########  ######    #######  ##     ##
//           ## ##        ##     ## ##       ##   ##   ##              ## ##     ##
//     ##    ## ##        ##     ## ##       ##    ##  ##       ##     ## ##     ##
//      ######  ##        ##     ## ######## ##     ## ########  #######  ########
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::sphere3d> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::sphere3d& msg) const{
            if (o.type != msgpack::type::MAP) throw msgpack::type_error();
            if (o.via.map.size != 2) throw msgpack::type_error();
            MAP_HASH_DEBUG("SPHERE3D");

            for(unsigned int ii=0; ii < o.via.map.size; ++ii){
                std::string key_str = o.via.map.ptr[ii].key.as<std::string>();
                // Const reference to val object, simplifies the code, adds to the readability
                const msgpack::object& val = o.via.map.ptr[ii].val;
                switch(std::hash<std::string>{}(key_str)){
                    case 17715334705756511795U:   // float64 radius
                        msg.radius = val.as<float64_t>(); break;
                    case 10907956872829995589U:   // point3d center
                        val.convert<pupil_msgs::point3d>(msg.center); break;
                }
            }

            return o;
        }
    };

//===========================================================================================================
//      ######                         ######
//      #     # #    # #####  # #      #     #   ##   ##### #    # #    #
//      #     # #    # #    # # #      #     #  #  #    #   #    # ##  ##
//      ######  #    # #    # # #      #     # #    #   #   #    # # ## #
//      #       #    # #####  # #      #     # ######   #   #    # #    #
//      #       #    # #      # #      #     # #    #   #   #    # #    #
//      #        ####  #      # ###### ######  #    #   #    ####  #    #
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::pupil_datum> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::pupil_datum& msg) const{
            if (o.type != msgpack::type::MAP) throw msgpack::type_error();
            if (o.via.map.size != 18) throw msgpack::type_error();

            MAP_HASH_DEBUG("PUPIL DATUM");

            for(unsigned int ii=0; ii < o.via.map.size; ++ii){
                const auto& T = o.via.map.ptr[ii];
                std::size_t hashed_key = std::hash<std::string>{}(T.key.as<std::string>());

                switch(hashed_key){
                case 9172301357109549112U:      // topic
                    msg.topic = T.val.as<std::string>(); break;
                case 54004706375334212U:        // method
                    msg.method = T.val.as<std::string>(); break;
                case 9282418322564408882U:      // norm_pos
                    T.val.convert<pupil_msgs::point2d>(msg.norm_pos); break;
                case 15108707956071064536U:     // diameter
                    msg.diameter = T.val.as<float64_t>(); break;
                case 17805607422174315343U:     // timestamp
                    msg.timestamp = T.val.as<float64_t>(); break;
                case 16657305780716913618U:     // confidence
                    msg.confidence = T.val.as<float64_t>(); break;
                case 11126994754711116986U:     // ellipse
                    T.val.convert<pupil_msgs::ellipse2d>(msg.ellipse); break;
                case 14258576900392064537U:     // id
                    msg.id = T.val.as<float64_t>(); break;
                case 5405470188062205385U:      // model_birth_timestamp
                    msg.model_birth_timestamp = T.val.as<float64_t>(); break;
                case 14873861199647364791U:     // model_confidence
                    msg.model_confidence = T.val.as<float64_t>(); break;
                case 3204079629990709446U:      // model_id
                    msg.model_id = T.val.as<float64_t>(); break;
                case 3139444447330049345U:      // theta
                    msg.theta = T.val.as<float64_t>(); break;
                case 2885876941129819837U:      // phi
                    msg.phi = T.val.as<float64_t>(); break;
                case 7174034577241048358U:      // circle_3d
                    T.val.convert<pupil_msgs::circle3d>(msg.circle_3d); break;
                case 2960994267254912948U:      // diameter_3d
                    msg.diameter_3d = T.val.as<float64_t>(); break;
                case 1006273858432046931U:      // sphere
                    T.val.convert<pupil_msgs::sphere3d>(msg.sphere); break;
                case 17945650170263818641U:     // projected_sphere
                    T.val.convert<pupil_msgs::ellipse2d>(msg.projected_sphere); break;
                case 4698864730501174006U:     // index
                    msg.index = T.val.as<uint64_t>(); break;
                default:
                    std::cout << "Unknown Key: " << T.key.as<std::string>() << std::endl; break;
                }
            }

            return o;
        }
    };

//===========================================================================================================
//       #####                       ######
//      #     #   ##   ###### ###### #     #   ##   ##### #    # #    #
//      #        #  #      #  #      #     #  #  #    #   #    # ##  ##
//      #  #### #    #    #   #####  #     # #    #   #   #    # # ## #
//      #     # ######   #    #      #     # ######   #   #    # #    #
//      #     # #    #  #     #      #     # #    #   #   #    # #    #
//       #####  #    # ###### ###### ######  #    #   #    ####  #    #
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::gaze_datum> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::gaze_datum& msg) const{
            if (o.type != msgpack::type::MAP) throw msgpack::type_error();
            if (o.via.map.size != 8) throw msgpack::type_error();
            MAP_HASH_DEBUG("GAZE DATUM");

            for(unsigned int kk=0; kk < o.via.map.size; ++kk){
                const auto& T = o.via.map.ptr[kk];
                std::size_t hashed_key = std::hash<std::string>{}(T.key.as<std::string>());

                switch(hashed_key){
                case 9172301357109549112U:   // topic
                    msg.topic = T.val.as<std::string>(); break;
                case 16657305780716913618U:  // confidence
                    msg.confidence = T.val.as<float64_t>(); break;
                case 9282418322564408882U:   // norm_pos
                    T.val.convert<pupil_msgs::point2d>(msg.norm_pos); break;
                case 17805607422174315343U:  // timestamp
                    msg.timestamp = T.val.as<float64_t>(); break;
                case 11444268017260055829U:  // gaze_normal_3d
                    T.val.convert<pupil_msgs::point3d>(msg.gaze_normal_3d); break;
                case 3572316667928812823U:   // eye_center_3d
                    T.val.convert<pupil_msgs::point3d>(msg.eye_center_3d); break;
                case 15529802796643644403U:  // gaze_point_3d
                    T.val.convert<pupil_msgs::point3d>(msg.gaze_point_3d); break;
                case 5329178820923943632U:   // base_data
                    // Test current base_data size, allocate only if necessary (next iterations wont reallocate)
                    if(msg.base_data.size() < T.val.via.array.size ){
                        std::cout << "Allocating base_data to " << T.val.via.array.size << std::endl;
                        msg.base_data.resize(T.val.via.array.size);
                    }
                    // Pass through the array and call convert function for each object
                    for(unsigned int ii=0; ii < T.val.via.array.size; ++ii){
                        T.val.via.array.ptr[ii].convert<pupil_msgs::pupil_datum>(msg.base_data[ii]);
                    }
                    break;
                default:
                    std::cout << "Unknown Key: " << T.key.as<std::string>() << std::endl; break;
                }

            }
            return o;
        }
    };


//===========================================================================================================
//     #### ##     ##    ###     ######   ########    #### ##    ## ########  #######
//      ##  ###   ###   ## ##   ##    ##  ##           ##  ###   ## ##       ##     ##
//      ##  #### ####  ##   ##  ##        ##           ##  ####  ## ##       ##     ##
//      ##  ## ### ## ##     ## ##   #### ######       ##  ## ## ## ######   ##     ##
//      ##  ##     ## ######### ##    ##  ##           ##  ##  #### ##       ##     ##
//      ##  ##     ## ##     ## ##    ##  ##           ##  ##   ### ##       ##     ##
//     #### ##     ## ##     ##  ######   ########    #### ##    ## ##        #######
//===========================================================================================================
    template<>
    struct convert<pupil_msgs::frame> {
        msgpack::object const& operator()(msgpack::object const& o, pupil_msgs::frame& msg) const{
            if (o.type != msgpack::type::MAP) throw msgpack::type_error();
            //if (o.via.map.size != 7) throw msgpack::type_error();
            //std::cout << "ALIVE" << std::endl;
            MAP_HASH_DEBUG("IMAGE INFO");

            //  [STRING] (9172301357109549112) topic
            //    [+INT] (12573571427077145850) height
            //    [+INT] (4698864730501174006) index
            // [FLOAT64] (17805607422174315343) timestamp
            //    [+INT] (10875507323944149648) width
            //  [STRING] (9264042919143940843) format

            for(unsigned int ii=0; ii < o.via.map.size; ++ii){
                std::string key_str = o.via.map.ptr[ii].key.as<std::string>();
                const msgpack::object& val = o.via.map.ptr[ii].val;
                switch(std::hash<std::string>{}(key_str)) {
                    case 9172301357109549112U:  // string topic
                        msg.topic = val.as<std::string>();
                        break;
                    case 4698864730501174006U:  // +int index
                        msg.index = val.as<uint64_t>();
                        break;
                    case 17805607422174315343U: // float64 timestamp
                        msg.timestamp = val.as<float64_t>();
                        break;
                    case 9264042919143940843U:  // string format
                        msg.format = val.as<std::string>();
                        break;
                    case 12573571427077145850U: // +int height
                        msg.height = val.as<uint64_t>();
                        break;
                    case 10875507323944149648U: // +int width
                        msg.width = val.as<uint64_t>();
                        break;
                    default:
                        std::cout << "Unknown Key: " << key_str << std::endl; break;
                }
            }

            return o;
        }
    };

}}}



#endif
