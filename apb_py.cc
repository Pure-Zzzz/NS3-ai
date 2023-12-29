/*
 * Copyright (c) 2023 Huazhong University of Science and Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors:  Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#include "apb.h"

#include <ns3/ai-module.h>

#include <iostream>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(ns3ai_apb_py_stru, m)
{
    py::class_<EnvStruct>(m, "PyEnvStruct")
        .def(py::init<>())
        .def_readwrite("current_channel", &EnvStruct::current_channel)
        .def_readwrite("current_power", &EnvStruct::current_power)
        .def_readwrite("current_disturbed_channel", &EnvStruct::current_disturbed_channel)
        .def_readwrite("current_snr", &EnvStruct::current_snr)
        .def_readwrite("envtmp4", &EnvStruct::envtmp4)
        .def_readwrite("envtmp3", &EnvStruct::envtmp3)
        .def_readwrite("envtmp2", &EnvStruct::envtmp2)
        .def_readwrite("id", &EnvStruct::id)
        .def_readwrite("cpp_action", &EnvStruct::cpp_action);

    py::class_<ActStruct>(m, "PyActStruct").def(py::init<>())
        .def_readwrite("next_channel", &ActStruct::next_channel)
        .def_readwrite("next_power", &ActStruct::next_power)
        .def_readwrite("acttmp1", &ActStruct::acttmp1)
        .def_readwrite("acttmp2", &ActStruct::acttmp2)
        .def_readwrite("acttmp3", &ActStruct::acttmp3)
        .def_readwrite("acttmp4", &ActStruct::acttmp4);

    py::class_<ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>>(m, "Ns3AiMsgInterfaceImpl")
        .def(py::init<bool,
                      bool,
                      bool,
                      uint32_t,
                      const char*,
                      const char*,
                      const char*,
                      const char*>())
        .def("PyRecvBegin", &ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>::PyRecvBegin)
        .def("PyRecvEnd", &ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>::PyRecvEnd)
        .def("PySendBegin", &ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>::PySendBegin)
        .def("PySendEnd", &ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>::PySendEnd)
        .def("PyGetFinished", &ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>::PyGetFinished)
        .def("GetCpp2PyStruct",
             &ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>::GetCpp2PyStruct,
             py::return_value_policy::reference)
        .def("GetPy2CppStruct",
             &ns3::Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>::GetPy2CppStruct,
             py::return_value_policy::reference);
}
