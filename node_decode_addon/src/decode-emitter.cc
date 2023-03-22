#include <chrono>
#include <iostream>
#include <thread>
#include <string.h>
#include "decode_interface.h"
#include "decode-emitter.h"
#include "common_func.h"
#include "common.h"
#include <windows.h>

#define MI_OUTPUT_FILE

int run_process(char* cmd_line)
{
    STARTUPINFO si = { sizeof(si) };
    PROCESS_INFORMATION pi;
    si.cb = sizeof(STARTUPINFO);
    si.dwFlags = STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_HIDE;
    // TCHAR command[] = TEXT(cmd_line);
    BOOL success = CreateProcess(NULL, cmd_line, NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi);
    if (success)
    {
        // 等待程序执行完成
        WaitForSingleObject(pi.hProcess, INFINITE);
        // 关闭进程和线程句柄
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
    }
    return 0;
}

Napi::FunctionReference decodeEmitter::constructor;

Napi::Object decodeEmitter::Init(Napi::Env env, Napi::Object exports) {
  Napi::Function func =
      DefineClass(env,
                  "decodeEmitter",
                  {InstanceMethod("SetExePath", &decodeEmitter::SetExePath),
                  InstanceMethod("SetMinDistanceLimit", &decodeEmitter::SetMinDistanceLimit),
                  InstanceMethod("StartStep", &decodeEmitter::StartStep),
                  InstanceMethod("EndStep", &decodeEmitter::EndStep),
                  InstanceMethod("ContinueStep", &decodeEmitter::ContinueStep),
                  InstanceMethod("InitIns401", &decodeEmitter::InitIns401),
                  InstanceMethod("InputIns401Buffer", &decodeEmitter::InputIns401Buffer),
                  InstanceMethod("DecodeIns401", &decodeEmitter::DecodeIns401),
                  InstanceMethod("InitRtk330la", &decodeEmitter::InitRtk330la),
                  InstanceMethod("InputRtk330laBuffer", &decodeEmitter::InputRtk330laBuffer),
                  InstanceMethod("DecodeRtk330la", &decodeEmitter::DecodeRtk330la),
                  InstanceMethod("SplitPostInsByTime", &decodeEmitter::SplitPostInsByTime),
                  InstanceMethod("CalcRoll", &decodeEmitter::CalcRoll),
                  InstanceMethod("CalcPitchHeading", &decodeEmitter::CalcPitchHeading)});

  constructor = Napi::Persistent(func);
  constructor.SuppressDestruct();

  exports.Set("decodeEmitter", func);
  return exports;
}

decodeEmitter::decodeEmitter(const Napi::CallbackInfo& info)
    : Napi::ObjectWrap<decodeEmitter>(info) {
  // NOOP
  m_Ins401_decoder = new Ins401_Tool::Ins401_decoder();
  m_Rtk330la_decoder = new RTK330LA_Tool::Rtk330la_decoder();
  m_SplitByTime = new SplitByTime();
  angle_list.clear();
  roll_list.clear();
  m_step = 0;
  m_step_running = false;
  m_ExePath.clear();
}

decodeEmitter::~decodeEmitter(){
  delete m_Ins401_decoder;
  m_Ins401_decoder = NULL;
  delete m_SplitByTime;
  m_SplitByTime = NULL;
}

Napi::Value decodeEmitter::SetExePath(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
    if (!info[0].IsString()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  m_ExePath = info[0].As<Napi::String>().ToString();
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::SetMinDistanceLimit(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
  if(!info[0].IsNumber())
  {
      Napi::Error::New(info.Env(), "Expected an Number").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  int min_distance_limit = info[0].As<Napi::Number>().Int32Value();
  m_SplitByTime->set_min_distance(min_distance_limit);
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::StartStep(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
  if(!info[0].IsNumber())
  {
      Napi::Error::New(info.Env(), "Expected an Number").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  m_step = info[0].As<Napi::Number>().Int32Value();
  m_step_running = true;
  m_SplitByTime->init();
  angle_list.clear();
  roll_list.clear();
  printf("StartStep %d\n",m_step);
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::EndStep(const Napi::CallbackInfo& info)
{
  // printf("EndStep\n");
  Napi::Env env = info.Env();
  m_step_running = false;
  m_SplitByTime->finish();
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::ContinueStep(const Napi::CallbackInfo& info){
  // printf("ContinueStep\n");
  Napi::Env env = info.Env();
  m_step_running = true;
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::InitIns401(const Napi::CallbackInfo& info)
{
  // printf("InitIns401\n");
  Napi::Env env = info.Env();
  m_Ins401_decoder->init();
  m_Ins401_decoder->set_output_file(false);
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::InputIns401Buffer(const Napi::CallbackInfo& info)
{
  // printf("InputIns401Buffer\n");
  Napi::Env env = info.Env();
    Napi::Function emit =
      info.This().As<Napi::Object>().Get("emit").As<Napi::Function>();
      
  if (!info[0].IsBuffer()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  Napi::Uint8Array buf = info[0].As<Napi::Uint8Array>();

  const unsigned char* buffer = reinterpret_cast<const unsigned char*>(buf.Data());
  size_t length = buf.ByteLength();
  int ret = 0,type = 0,split_ret = 0;
  if (m_Ins401_decoder) {
    for(size_t i =0;i < length; i++){
      ret = m_Ins401_decoder->input_data(buffer[i]);
      if(ret){
        type = m_Ins401_decoder->get_current_type();
        // emit.Call(info.This(),
        //       {Napi::String::New(env, "datatype"),Napi::Number::New(env,type)});
        if(m_step_running){
          if(Ins401_Tool::em_INS_SOL == type){
            struct ins_sol_data ins_data = { 0 };
						Ins401_Tool::ins_sol_t* ins_p = m_Ins401_decoder->get_ins_sol();
						ins_data.gps_week = ins_p->gps_week;
						ins_data.gps_millisecs = ins_p->gps_millisecs;
						ins_data.ins_status =ins_p->ins_status;
						ins_data.ins_position_type = ins_p->ins_position_type;
						ins_data.latitude = ins_p->latitude;
						ins_data.longitude = ins_p->longitude;
						ins_data.height = ins_p->height;
						ins_data.north_velocity = ins_p->north_velocity;
						ins_data.east_velocity = ins_p->east_velocity;
						ins_data.up_velocity = ins_p->up_velocity;
						ins_data.roll = ins_p->roll;
						ins_data.pitch = ins_p->pitch;
						ins_data.heading = ins_p->heading;
            split_ret = m_SplitByTime->input_sol_data(ins_data);
            if(split_ret >= 0){
              struct stTimeSlice* slice = m_SplitByTime->get_current_slice();
              if(split_ret == 1){
                if(env,slice->angle_diff != 0){
                  TurnInfo_t turn = {0};
                  turn.slice = m_SplitByTime->get_time_slices().size();
                  turn.time = slice->starttime/1000;
                  turn.speed = slice->currentspeed;
                  turn.angle = slice->angle_diff;
                  emit.Call(info.This(),{Napi::String::New(env, "onTurn"),Napi::Buffer<char>::Copy(env,(char*)&turn,sizeof(TurnInfo_t))});
                }
              }
              if(slice->starttime != 0){
                Distance_t distance = {0};
                distance.slice = m_SplitByTime->get_time_slices().size();
                distance.time = slice->currenttime/1000;
                distance.speed = slice->currentspeed;
                distance.distance = slice->speed_distance;
                emit.Call(info.This(),{Napi::String::New(env, "onDistance"),Napi::Buffer<char>::Copy(env,(char*)&distance,sizeof(Distance_t))});
              }
            }
          }
        }
      }
    }
  }
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::DecodeIns401(const Napi::CallbackInfo& info){
  Napi::Env env = info.Env();
  if (!info[0].IsString()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  char file_name[1024] = {0};
  std::string filename = info[0].As<Napi::String>().ToString();
  strcpy(file_name,filename.c_str());
  char is_parse_dr[] = "false";
	decode_ins401_interface(file_name, is_parse_dr);
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::InitRtk330la(const Napi::CallbackInfo& info)
{
  // printf("InitIns401\n");
  Napi::Env env = info.Env();
  m_Rtk330la_decoder->init();
  m_Rtk330la_decoder->set_output_file(false);
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::InputRtk330laBuffer(const Napi::CallbackInfo& info)
{
    Napi::Env env = info.Env();
    Napi::Function emit =
      info.This().As<Napi::Object>().Get("emit").As<Napi::Function>();
      
  if (!info[0].IsBuffer()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  Napi::Uint8Array buf = info[0].As<Napi::Uint8Array>();

  const unsigned char* buffer = reinterpret_cast<const unsigned char*>(buf.Data());
  size_t length = buf.ByteLength();
  int ret = 0,type = 0,split_ret = 0;
  if (m_Rtk330la_decoder) {
    for(size_t i =0;i < length; i++){
      ret = m_Rtk330la_decoder->input_raw(buffer[i]);
      if(ret){
        type = m_Rtk330la_decoder->get_current_type();
        if(m_step_running){
          if(RTK330LA_Tool::em_iN == type){
            ins_sol_data ins_data = { 0 };
						RTK330LA_Tool::inceptio_iN_t* ins_p = m_Rtk330la_decoder->get_ins_sol();
						ins_data.gps_week = ins_p->GPS_Week;
						ins_data.gps_millisecs = uint32_t(ins_p->GPS_TimeOfWeek * 1000);
						ins_data.ins_status = ins_p->insStatus;
						ins_data.ins_position_type = ins_p->insPositionType;
						ins_data.latitude = (double)ins_p->latitude *180.0 / MAX_INT;
						ins_data.longitude = (double)ins_p->longitude *180.0 / MAX_INT;
						ins_data.height = ins_p->height;
						ins_data.north_velocity = (float)ins_p->velocityNorth / 100.0;
						ins_data.east_velocity = (float)ins_p->velocityEast / 100.0;
						ins_data.up_velocity = (float)ins_p->velocityUp / 100.0;
						ins_data.roll = (float)ins_p->roll / 100.0;
						ins_data.pitch = (float)ins_p->pitch / 100.0;
						ins_data.heading = (float)ins_p->heading / 100.0;
            split_ret = m_SplitByTime->input_sol_data(ins_data);
            if(split_ret >= 0){
              struct stTimeSlice* slice = m_SplitByTime->get_current_slice();
              if(split_ret == 1){
                if(slice->angle_diff != 0){
                  TurnInfo_t turn = {0};
                  turn.slice = m_SplitByTime->get_time_slices().size();
                  turn.time = slice->starttime/1000;
                  turn.speed = slice->currentspeed;
                  turn.angle = slice->angle_diff;
                  emit.Call(info.This(),{Napi::String::New(env, "onTurn"),Napi::Buffer<char>::Copy(env,(char*)&turn,sizeof(TurnInfo_t))});
                }
              }
              if(slice->starttime != 0){
                Distance_t distance = {0};
                distance.slice = m_SplitByTime->get_time_slices().size();
                distance.time = slice->currenttime/1000;
                distance.speed = slice->currentspeed;
                distance.distance = slice->speed_distance;
                emit.Call(info.This(),{Napi::String::New(env, "onDistance"),Napi::Buffer<char>::Copy(env,(char*)&distance,sizeof(Distance_t))});
              }
            }
          }
        }
      }
    }
  }
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::DecodeRtk330la(const Napi::CallbackInfo& info){
  Napi::Env env = info.Env();
  if (!info[0].IsString()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  char file_name[1024] = {0};
  std::string filename = info[0].As<Napi::String>().ToString();
  strcpy(file_name,filename.c_str());
	decode_rtk330la_interface(file_name);
  return Napi::String::New(env, "OK");
}

void readRollFromIns(FILE* f_ins,std::vector<stTimeSlice>& time_slices,std::vector<stAngle>& angle_list) {
	if (f_ins) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double sum_roll = 0.0;
		int roll_num = 0;
    int max_slices = time_slices.size();
    if(time_slices.size() % 2 == 1){
      max_slices--; 
    }
    if(max_slices <= 0){
      return;
    }
    for(int i = 0;i < time_slices.size(); i++){
      printf("angle_diff = %f\n",time_slices[i].angle_diff);
    }
		while (fgets(line, 256, f_ins) != NULL) {
			line_num++;
			if (line_num == 1) continue;
      string line_str(line);
      // printf(line);
      vector<string> items = split(line_str,",");
			if (items.size() < 13) continue;
      uint32_t time = std::stof(items[1])*1000;
			if (time < time_slices[time_index].starttime)
				continue;
			if (time > time_slices[time_index].endtime) {
        if (roll_num > 0) {
          stAngle angle = { 0 };
          angle.roll = sum_roll / roll_num;
          angle_list.push_back(angle);
        }
				time_index++;
				sum_roll = 0.0;
				roll_num = 0;
				if (time_index >= max_slices)
					break;
			}
			if (time % 1000 != 0)
				continue;
			if (time >= time_slices[time_index].starttime && time <= time_slices[time_index].endtime) {
				double roll = std::stof(items[8]);
				sum_roll += roll;
				roll_num++;
			}
		}
	}
}

Napi::Value decodeEmitter::SplitPostInsByTime(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
  if (!info[0].IsString()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  char file_name[1024] = {0};
  string filename = info[0].As<Napi::String>().ToString();
  strcpy(file_name,filename.c_str());
  FILE* f_post_ins = fopen(filename.c_str(), "r");
  char delim[] = ",";//分隔符字符串
  char line[512] = {0};
  if(f_post_ins){
    m_SplitByTime->init();
    while(fgets(line,512,f_post_ins) != NULL){
      // while(line[strlen(line)-1] == '\n' || line[strlen(line)-1] == '\r'){
      //   line[strlen(line)-1] == 0;
      // }
      double value_list[23] = { 0 };
      split2double(line,delim,value_list,23);
      ins_sol_data ins_data = { 0 };
      ins_data.gps_week = (uint16_t)value_list[0];
      ins_data.gps_millisecs = (uint32_t)(value_list[1]*1000);
      ins_data.ins_status = (uint8_t)value_list[19];
      ins_data.ins_position_type = (uint8_t)value_list[20];
      ins_data.latitude = value_list[2];
      ins_data.longitude = value_list[3];
      ins_data.height = value_list[4];
      ins_data.north_velocity = value_list[5];
      ins_data.east_velocity = value_list[6];
      ins_data.up_velocity = value_list[7];
      ins_data.roll = value_list[8];
      ins_data.pitch = value_list[9];
      ins_data.heading = value_list[10];
      m_SplitByTime->input_sol_data(ins_data);
    }
    m_SplitByTime->finish();
    fclose(f_post_ins);
  }
  return Napi::String::New(env, "OK");
}

Napi::Value decodeEmitter::CalcRoll(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
  if (!info[0].IsString()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  char file_name[1024] = {0};
  std::string filename = info[0].As<Napi::String>().ToString();
  strcpy(file_name,filename.c_str());
  std::vector<stTimeSlice>& time_slices = m_SplitByTime->get_time_slices();
  stAngle avg_angle = {0};
  if(time_slices.size() > 0){
    FILE* f_ins = fopen(filename.c_str(), "r");
    readRollFromIns(f_ins,time_slices,roll_list);
    fclose(f_ins);
    printf("%lld\n",roll_list.size());
    //calculate average angle    
    stAngle sum_angle = {0};
    if(roll_list.size() > 0){
      for(int i = 0;i < roll_list.size();i++){
        sum_angle.roll += roll_list[i].roll;
      }
      avg_angle.roll = sum_angle.roll/ roll_list.size();
    }
  }
  return Napi::Number::New(env, avg_angle.roll);
}

void findNumStr(char*dst, char* src,int len){
  int j = 0;
  for(int i = 0;i < len;i++){
    if(src[i] == ' ' || src[i] == '\r'|| src[i] == '\n'|| src[i] == ',') continue;
    dst[j] = src[i];
    j++;
  }
}

int readAngleFromFile(const char* file_path,stAngle& angle)
{
	FILE* f_out = fopen(file_path, "r");
	if (f_out) {
		char line[256] = { 0 };
		int index = 0;
		while (fgets(line, 256, f_out) != 0){
			index++;
			if (index == 2) {
        char num_str[32] = {0};
				findNumStr(num_str,line,strlen(line));
				angle.roll = atof(num_str);
			}
			if (index == 3) {
        char num_str[32] = {0};
				findNumStr(num_str,line,strlen(line));
				angle.pitch = atof(num_str);
			}
			if (index == 4) {
				char num_str[32] = {0};
				findNumStr(num_str,line,strlen(line));
				angle.heading = atof(num_str);
			}
		}
		fclose(f_out);
    return 0;
	}
  return 1;
}

Napi::Value decodeEmitter::CalcPitchHeading(const Napi::CallbackInfo& info)
{
  Napi::Env env = info.Env();
  if (!info[0].IsString()) 
  {
      Napi::Error::New(info.Env(), "Expected an Buffer").ThrowAsJavaScriptException();
      return info.Env().Undefined();
  }
  char file_name[1024] = {0};
  char file_dir[1024] = {0};
  std::string filename = info[0].As<Napi::String>().ToString();
  strcpy(file_name,filename.c_str());
  char* p = strrchr(file_name,'\\');
  strncpy(file_dir,file_name,p-file_name);

  std::vector<stTimeSlice>& time_slices = m_SplitByTime->get_time_slices();
  for(int i = 0; i < time_slices.size();i++){
    int  start_time = time_slices[i].starttime/1000;
    int  end_time = time_slices[i].endtime/1000;
    int week = time_slices[i].week;
    char cmd[1024] = {0};
    char outprefix[1024] = {0};
    char resultfilePath[1024] = {0};
    std::string exefilePath = m_ExePath + "\\solveMisalign.exe";
    sprintf(outprefix,"%s\\misalign_%d-%d",file_dir,start_time,end_time);
    sprintf(cmd,"%s -t \"%s\" -o \"%s\"  -rng %d %d %d", exefilePath.c_str(), file_name, outprefix, start_time, end_time, week);
    run_process(cmd);
    // system(cmd);
    sprintf(resultfilePath,"%s%s",outprefix,"_content_misalign.txt");
    stAngle angle = { 0 };
    int try_count = 0;
    while(readAngleFromFile(resultfilePath,angle)){
      std::this_thread::sleep_for(std::chrono::seconds(1));
      try_count++;
      if(try_count >= 10){
        break;
      }
    }
    angle_list.push_back(angle);
  }
  //calculate average angle
  stAngle avg_angle = {0};
  stAngle sum_angle = {0};
  if(angle_list.size() > 0){
    for(int i = 0;i < angle_list.size();i++){
      sum_angle.heading += angle_list[i].heading;
      sum_angle.pitch += angle_list[i].pitch;
    }
    avg_angle.heading = sum_angle.heading/ angle_list.size();
    avg_angle.pitch = sum_angle.pitch/ angle_list.size();
  }
  return Napi::Buffer<char>::Copy(env,(char*)&avg_angle,sizeof(stAngle));
}