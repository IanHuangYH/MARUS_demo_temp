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
    public class gripper : SensorStreamer<SensorStreamingClient, DvlStreamingRequest>
    {
        DvlSensor sensor;
        Thread _handleStreamThread;
        ServerStreamer<ForceResponse> _streamer;
        private ArticulationBody ab;
        List<float> pos_act, path;
        List<float> vel_act;
        public float torque, freq=0f, mode, pos_des, vel_des, saturation=1f, ilimit=1f, p=0.1f, i=0f, d=0f, wp=0f;
        float delta_time, delta_wp=0f, last_time_stamp, delta, control_signal, error, previous_error_ = 0f, previous_time_ = -1f, integrated_error_ = 0f, last_pos_des;
        UnityEngine.Vector3 pub_contr_sig = new UnityEngine.Vector3(0.0f,0.0f,0.0f);
        UnityEngine.Vector3 pub_contr_sig_2 = new UnityEngine.Vector3(0.0f,0.0f,0.0f);
        
        int k=0;

        // Start is called before the first frame update
        void Awake()
        {
            
            sensor = GetComponent<DvlSensor>();
            StreamSensor(sensor,
                streamingClient.StreamDvlSensor);
            base.Start();

            ab = GetComponent<ArticulationBody>();
            
            pos_act = new List<float>(2);
            vel_act = new List<float>(2);
            delta = 0.08f;


            // _targetTransform = transform;
            // _streamer = new ServerStreamer<ForceResponse>(UpdateMovement);
            // var client = RosConnection.Instance.GetClient<RemoteControlClient>();
            // var address = Helpers.GetVehicle(transform)?.name ?? name;
            // _streamer.StartStream(client.ApplyForce(
            //     new ForceRequest { Address = $"{gameObject.name}/toogle_jaw" },
            //     cancellationToken: RosConnection.Instance.CancellationToken)
            // );
        }

        //Gripper movement is jumpy - due to collision or torques induced from each gripper link --> disable collision for smooth operation
        void Update()
        {
            int i = ab.index+5;
            ab.GetJointPositions(pos_act);
            ab.GetJointVelocities(vel_act);
            delta_time = (Time.time - last_time_stamp);
            last_time_stamp = Time.time;

            if (pos_des != last_pos_des)//new goal
            {
                Debug.Log("Planning");
                wp=pos_act[i];
                delta_wp=(pos_des-pos_act[i])/200; //divide into path
                k=0;
                last_pos_des=pos_des;
            }
            
            if(mode == 0) { //stupid torque publish
                if(delta_time > freq) {
                    publish_torque(torque);
                    last_time_stamp = Time.time;
                }
            }
            if(mode != 0) {
                
                if(mode == 1) { //Position control

                    error = pos_des-pos_act[i];//wp-pos_act[i];

                    if(Math.Abs(error) < 0.01 && k < 199) {
                        wp = wp+delta_wp;
                        Debug.Log("wp: " + wp);
                        k++;
                    }
                } 
                if (mode == 2) { //Velocity control
                    error = vel_des-vel_act[i];
                }

                control_signal = regulate(error, Time.time);
                pub_contr_sig[0] = control_signal;
                if(ab.index == 1){//Gripper1
                    pub_contr_sig[1] = (float)(-pos_des-1.57);
                } else { //Gripper2
                    pub_contr_sig[1] = (float)(1.57-pos_des);//(float)(-wp+1.57);
                }
                
                publish_torque(control_signal);
            }
            Debug.Log("2: " + pos_act[6] + " 1: " + pos_act[7]);
            // Debug.Log(ab.index);
            // Debug.Log("pos_act: A: " + pos_act[6]);// + ", B: " + pos_act[1]);
            // Debug.Log("vel_act: A: " + vel_act[6]);// + ", B: " + vel_act[1]);
            // // _streamer.HandleNewMessages();
        }
        
        void publish_torque(float control_signal) {
            ab.AddTorque(transform.right * control_signal);
        }
        float regulate(float error, float time)
        {
            float delta_time = time - previous_time_;

            float error_p = error;
            float error_d = 0;


            if (delta_time != 0) {
                error_d = (error - previous_error_)/delta_time;
            } 

            float error_i = (float)(integrated_error_ + 0.5*(error + previous_error_)*delta_time);

            if(error_i > ilimit)
            {
                error_i = ilimit;
            }
            else if(error_i < -ilimit)
            {
                error_i = -ilimit;
            }

            float control_signal_ = p*error_p + d*error_d + i*error_i;

            if(control_signal_ > saturation)
            {
                control_signal_ = saturation;
            }
            else if(control_signal_ < -saturation)
            {
                control_signal_ = -saturation;
            }

            previous_error_ = error;
            previous_time_ = time;
            integrated_error_ = error_i;

            return control_signal_;
        }

        protected override DvlStreamingRequest ComposeMessage()//compare WAYPOINT (/colrov_skid/skid_jaw_1/twist/twist/linear/z) & ACTUAL POSITION (/tf/base_link/skid_jaw_1/rotation/pitch)
        {
            var dvlOut = new TwistWithCovarianceStamped
            {
                Header = new Header()
                {
                    FrameId = sensor.frameId,
                    Timestamp = TimeHandler.Instance.TimeDouble
                },
                Twist = new TwistWithCovariance
                {
                    Twist = new Twist
                    {
                        Linear = pub_contr_sig.Unity2Body().AsMsg(),//sensor.groundVelocity.Unity2Body().AsMsg()
                        Angular = pub_contr_sig_2.Unity2Body().AsMsg()
                    }
                }
            };
            dvlOut.Twist.Covariance.AddRange(sensor.velocityCovariance);

            return new DvlStreamingRequest
            {
                Address = address,
                Data = dvlOut
            };
            // await _streamWriter.WriteAsync(request);
        }

        // void UpdateMovement() { //use with InvokeRepeat - undefineable frequency & performance issues
        //     
        // }

        // void UpdateMovement(ForceResponse result)
        // {
        //     var hinge = GetComponent<HingeJoint>();
        //     var motor = hinge.motor;
        //     var limits = hinge.limits;
        //     var targetVelocity = motor.targetVelocity;

        //     if(targetVelocity == 0) {
        //         if(limits.max == 0) //skid_jaw_2
        //             motor.targetVelocity = -200;
        //         else //skid_jaw_1
        //             motor.targetVelocity = 200;
        //     }
        //     else
        //         motor.targetVelocity *= -1;
        //     hinge.motor = motor;
        //     // thrusterController.ApplyInput(result.Pwm.Data.ToArray());
        // }
    }
}