'use strict'

const EventEmitter = require('events').EventEmitter
const decode_Emitter = require('bindings')('decode_emitter').decodeEmitter
const inherits = require('util').inherits
const ref = require('ref-napi')
const StructType = require('ref-struct-di')(ref)

inherits(decode_Emitter, EventEmitter)

var TurnInfo_t = StructType(
    {
        slice : ref.types.uint32,
        time : ref.types.uint32,
        angle : ref.types.double,
        speed : ref.types.double
    }
);
var Distance_t = StructType(
    {
        slice : ref.types.uint32,
        time : ref.types.uint32,
        distance : ref.types.double,
        speed : ref.types.double
    }
);

var Angle_t = StructType(
    {
        roll : ref.types.float,
        pitch : ref.types.float,
        heading : ref.types.float
    }
);

class DecoderEmitter extends EventEmitter {
    constructor(){
        super()
        this.emitter =  new decode_Emitter()
        this.Run();
    }

    Run(){
        this.emitter.on('onDistance', (evt) => {
            var distance = new Distance_t(evt);
            var out_data = {
                slice : distance.slice,
                time : distance.time,
                distance : distance.distance,
                speed : distance.speed
            }
            // console.log('onDistance',out_data)
            this.emit("Distance",out_data);
        })
        this.emitter.on('onTurn', (evt) => {
            var turn = new TurnInfo_t(evt);
            var out_data = {
                slice : turn.slice,
                time : turn.time,
                angle : turn.angle,
                speed : turn.speed
            }
            // console.log('onTurn',out_data);
            this.emit("Turn",out_data);
        })
    }

    SetExePath(path){
        this.emitter.SetExePath(path);
    }

    SetMinDistanceLimit(distance){
        this.emitter.SetMinDistanceLimit(distance);
    }

    StartStep(step){
        this.emitter.StartStep(step);
    }   

    EndStep(){
        this.emitter.EndStep();
    }   

    InitIns401(){
        this.emitter.InitIns401();
    }
    
    InputIns401Buffer(data){
        this.emitter.InputIns401Buffer(data);
    }


    DecodeIns401(filename){
        this.emitter.DecodeIns401(filename);
    }   
    
    InitRtk330la(){
        this.emitter.InitRtk330la();
    }
    
    InputRtk330laBuffer(data){
        this.emitter.InputRtk330laBuffer(data);
    }

    DecodeRtk330la(filename){
        this.emitter.DecodeRtk330la(filename);
    } 

    SplitPostInsByTime(filename){
        console.log("SplitPostInsByTime:",filename);
        this.emitter.SplitPostInsByTime(filename);
    }

    CalcPitchHeading(process_ins_txt){
        var result = this.emitter.CalcPitchHeading(process_ins_txt);
        var angle = new Angle_t(result)
        var out_data = {
            pitch : angle.pitch,
            heading : angle.heading
        }
        return out_data;
    }

    CalcRoll(ins_txt){
        var roll = this.emitter.CalcRoll(ins_txt);
        return roll;
    }
};

module.exports = DecoderEmitter;
