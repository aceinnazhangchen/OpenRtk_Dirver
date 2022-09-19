'use strict'

const DecoderEmitter = require('./index')
const fsPromises = require("fs/promises")
const fs = require("fs")

// const ins401_filename = "F:\\data\\2022\\178\\session_20220627_105149\\ins401_log_2209000435_20220627_105208\\user_2022_06_27_10_52_19.bin"
// const ins401_ins_txt = "F:\\data\\2022\\178\\session_20220627_105149\\ins401_log_2209000435_20220627_105208\\user_2022_06_27_10_52_19_d\\user_2022_06_27_10_52_19_ins.txt"
// const ins401_porcess_ins = "F:\\data\\2022\\178\\session_20220627_105149\\ins401_log_2209000435_20220627_105208\\user_2022_06_27_10_52_19_d\\user_2022_06_27_10_52_19_process_ins.txt"

const rtk330la_filename = "F:\\data\\2022\\178\\session_20220627_105149\\rtk330la_log_2178200286_20220627_105208\\user_2022_06_27_10_52_26.bin"
const rtk330la_ins_txt = "F:\\data\\2022\\178\\session_20220627_105149\\rtk330la_log_2178200286_20220627_105208\\user_2022_06_27_10_52_26_d\\user_2022_06_27_10_52_26_ins.txt"
const rtk330la_porcess_ins = "F:\\data\\2022\\178\\session_20220627_105149\\rtk330la_log_2178200286_20220627_105208\\user_2022_06_27_10_52_26_d\\user_2022_06_27_10_52_26_process_ins.txt"

const emitter = new DecoderEmitter()

emitter.on('Distance', (evt) => {
    console.log('onDistance',"id:",evt.slice,"time:",evt.time,"distance:",evt.distance,"speed:",evt.speed)
})

emitter.on('Turn', (evt) => {
    console.log('onTurn',"id:",evt.slice,"time:",evt.time,"angle:",evt.angle,"speed:",evt.speed);
})


function DecodeIns401(file_name){
    emitter.DecodeIns401(file_name)
}

function DecodeRtk330la(file_name){
    emitter.DecodeRtk330la(file_name)
}

function calc_roll(file_name){
    var roll = emitter.CalcRoll(file_name);
    console.log(roll);
}

function calc_heading(file_name){
    var heading = emitter.CalcPitchHeading(file_name);
    console.log(heading)
}

const ins401_filename = "F:\\data\\user_2022_08_30_11_13_29.bin"

function test_file_ins401(){
    emitter.SetExePath("F:\\OpenRtk_Dirver\\node_decode_addon\\");
    emitter.InitIns401();
    emitter.StartStep(1);
    emitter.SetMinDistanceLimit(150);
    const reader = fs.createReadStream(ins401_filename, {
        highWaterMark: 100
      });
    
    reader.on('data', (data) => {
        emitter.InputIns401Buffer(data);
    });

    reader.on('close', () => {
        console.log('user.bin close');
        emitter.EndStep();
        DecodeIns401(ins401_filename);
        // calc_roll(ins401_ins_txt);
        // calc_heading(ins401_porcess_ins);
    });
}

// const rtk330la_filename = "F:\\data\\user_2022_08_25_13_58_39.bin";
// const rtk330la_ins_txt = "F:\\data\\user_2022_08_25_13_58_39_d\\user_2022_08_25_13_58_39_ins.txt";

function test_file_rtk330la(){
    emitter.SetExePath("F:\\OpenRtk_Dirver\\node_decode_addon\\");
    emitter.InitRtk330la();
    emitter.StartStep(1);
    const reader = fs.createReadStream(rtk330la_filename, {
        highWaterMark: 100
      });
    
    reader.on('data', (data) => {
        emitter.InputRtk330laBuffer(data);
    });

    reader.on('close', () => {
        console.log('user.bin close');
        emitter.EndStep();
        DecodeRtk330la(rtk330la_filename);
        calc_roll(rtk330la_ins_txt);
        calc_heading(rtk330la_porcess_ins);
    });
}

// test_file_rtk330la();
test_file_ins401();