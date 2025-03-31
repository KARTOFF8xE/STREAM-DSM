#include "datamgmt/datamgmt.hpp"


namespace relationMgmt {

/**
 * @brief Manages the creation of namespace- and process trees and links them with passive helpers.
 * // TODO
 * @param response A NodeResponse.
 */
void relationMgmt(std::map<Module_t, Pipe> pipes, std::atomic<bool> &running);

}