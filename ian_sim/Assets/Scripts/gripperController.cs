// Copyright 2022 Laboratory for Underwater Systems and Technologies (LABUST)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;

using Remotecontrol;
using System.Threading;
using Marus.Networking;
using System.Linq;
using static Remotecontrol.RemoteControl;
using Marus.Utils;
using System.Collections;
using System.Collections.Generic;
using System;
using Geometry;
using Marus.Core;
using Marus.Sensors.Primitive;
using Sensorstreaming;
using Std;
using static Sensorstreaming.SensorStreaming;

namespace Marus.Sensors.ROS
{
    public class gripperController : MonoBehaviour
    {
        public float open_gripper = 0f,p=1,i=0,d=0,sat=1,ilimit=1;

        void Awake()
        {
            
        }
        
        void Update()
        {
            GameObject jaw1 = GameObject.Find("skid_jaw_1");
            gripper cs1 = jaw1.GetComponent<gripper>();
            float jaw1_pos = cs1.pos_des;

            GameObject jaw2 = GameObject.Find("skid_jaw_2");
            gripper cs2 = jaw2.GetComponent<gripper>();
            float jaw2_pos = cs2.pos_des;

            if(open_gripper == 1){
                cs1.pos_des = 0;
                cs2.pos_des = 0;
            } else {
                cs1.pos_des = -1.57f;
                cs1.p = p;
                cs1.i = i;
                cs1.d = d;
                cs1.saturation = sat;
                cs1.ilimit = ilimit;

                cs2.pos_des = 1.57f;
                cs2.p = p;
                cs2.i = i;
                cs2.d = d;
                cs2.saturation = sat;
                cs2.ilimit = ilimit;

            }
        }
    }
}