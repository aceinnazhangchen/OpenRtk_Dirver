#include "MergeThread.h"
#include <QDir>
#include <QDebug>

MergeThread::MergeThread(QObject *parent)
	: QThread(parent)
	, m_isStop(true)
	, m_MergeFormat(emMergeFromat_rtcm_imu)
	, m_outlen(0)
	, m_rtcmFile(NULL)
	, m_imuFile(NULL)
	, m_roverFile(NULL)
	, m_baseFile(NULL)
	, m_outFile(NULL)
	, m_logFile(NULL)
	, m_RawFilesSize(0)
	, m_RawFilesReadSize(0)
	, last_TimeOfWeek(0)
{
	memset(m_outbuff, 0, MAX_BUFFER_SIZE);
	memset(&m_gnss, 0, sizeof(m_gnss));
}

MergeThread::~MergeThread()
{
}

void MergeThread::run()
{
	m_isStop = false;
	if (!m_MergeFileName1.isEmpty() && !m_MergeFileName2.isEmpty()) {
		m_TimeCounter.start();
		m_RawFilesReadSize = 0;
		makeOutFileName();
		switch (m_MergeFormat)
		{
		case emMergeFromat_rtcm_imu:
			mergeRtcmImuFile();
			break;
		case emMergeFromat_rover_base:
			mergeRoverBaseFile();
			break;
		default:
			break;
		}
	}
	emit sgnFinished();
}

void MergeThread::stop()
{
	m_isStop = true;
}

void MergeThread::setMergeFormat(int format)
{
	m_MergeFormat = format;
}

void MergeThread::setMergeFileName1(QString file)
{
	m_MergeFileName1 = file;
}

void MergeThread::setMergeFileName2(QString file)
{
	m_MergeFileName2 = file;
}

void MergeThread::makeOutFileName()
{
	QFileInfo outDir(m_MergeFileName1);
	if (outDir.isFile()) {
		if (m_MergeFormat == emMergeFromat_rtcm_imu) {
			QString basename = outDir.baseName();
			QString absoluteDir = outDir.absoluteDir().absolutePath();
			m_OutFileName = absoluteDir + QDir::separator() + basename + ".imu-rtcm";
			m_OutLogName = absoluteDir + QDir::separator() + basename + ".imu-rtcm.log";

		}
		else if (m_MergeFormat == emMergeFromat_rover_base) {
			QString basename = outDir.baseName();
			QString absoluteDir = outDir.absoluteDir().absolutePath();
			m_OutFileName = absoluteDir + QDir::separator() + basename + ".rover-base";
			m_OutLogName = absoluteDir + QDir::separator() + basename + ".rover-base.log";
		}
	}
}

void MergeThread::mergeRtcmImuFile() {
	if (is_aceinna_decoding()) return;
	set_aceinna_decoding(1);
	m_rtcmFile = fopen(m_MergeFileName1.toLocal8Bit().data(),"rb");
	m_imuFile = fopen(m_MergeFileName2.toLocal8Bit().data(), "rb");
	m_outFile = fopen(m_OutFileName.toLocal8Bit().data(), "wb");
	m_logFile = fopen(m_OutLogName.toLocal8Bit().data(), "w");
	if (m_rtcmFile && m_imuFile && m_outFile && m_logFile) {
		m_RawFilesSize = getFileSize(m_rtcmFile);
		m_RawFilesSize += getFileSize(m_imuFile);
		m_roverbuff.clear();
		m_basebuff.clear();
		m_imubuff.clear();
		memset(&m_gnss, 0, sizeof(m_gnss));
		int stn = 0;
		while (!feof(m_rtcmFile) && !feof(m_imuFile)) {
			if (m_isStop) break;
			int ret = readOneImu();
			user_s1_t* pak = getImuPak();
			if (ret = 1 && pak->GPS_TimeOfWeek % 100 == 0) {
				while (!feof(m_rtcmFile)) {
					if (m_isStop) break;
					if(TYPE_ROV == stn && m_roverbuff.size() > 0){
						//last rtcm is later
					}
					else {
						stn = readOneRtcm();
					}
					if (TYPE_ROV == stn) {
						int week = 0;
						double second = time2gpst(m_gnss.rcv[0].time, &week);
						uint32_t nsec = second * 1000;
						if (week == pak->GPS_Week) {
							if (last_TimeOfWeek <= nsec && nsec <= pak->GPS_TimeOfWeek) {
								fwrite(m_roverbuff.data(), 1, m_roverbuff.size(), m_outFile);
								fprintf(m_logFile, "rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
								m_roverbuff.clear();
								break;
							}
							else if (nsec > pak->GPS_TimeOfWeek) {
								//fprintf(m_logFile, "later rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
								break;
							}
							else {
								fprintf(m_logFile, "early rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
								m_roverbuff.clear();
							}
						}
						else {
							fprintf(m_logFile, "week error rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
							m_roverbuff.clear();
						}
					}
					else if (TYPE_BAS == stn) {
						int week = 0;
						double second = time2gpst(m_gnss.rcv[1].time, &week);
						uint32_t nsec = second * 10000;
						fwrite(m_basebuff.data(), 1, m_basebuff.size(), m_outFile);
						fprintf(m_logFile, "base,week:%d,sec:%d,size:%d\n", week, nsec, m_basebuff.size());
						m_basebuff.clear();
					}
				}
			}
		}
	}
	if (m_rtcmFile) fclose(m_rtcmFile); m_rtcmFile = NULL;
	if (m_imuFile) fclose(m_imuFile); m_imuFile = NULL;
	if (m_outFile) fclose(m_outFile); m_outFile = NULL;
	if (m_logFile) fclose(m_logFile); m_logFile = NULL;
	set_aceinna_decoding(0);
}

void MergeThread::mergeRoverBaseFile()
{
	m_roverFile = fopen(m_MergeFileName1.toLocal8Bit().data(), "rb");
	m_baseFile = fopen(m_MergeFileName2.toLocal8Bit().data(), "rb");
	m_outFile = fopen(m_OutFileName.toLocal8Bit().data(), "wb");
	m_logFile = fopen(m_OutLogName.toLocal8Bit().data(), "w");
	if (m_roverFile && m_baseFile && m_outFile && m_logFile) {
		m_RawFilesSize = getFileSize(m_roverFile);
		m_RawFilesSize += getFileSize(m_baseFile);
		m_roverbuff.clear();
		m_basebuff.clear();
		memset(&m_gnss, 0, sizeof(m_gnss));
		while (!feof(m_roverFile) && !feof(m_baseFile)) {
			readOneRover();
			readOneBase();
		}
	}
	if (m_roverFile) fclose(m_roverFile); m_roverFile = NULL;
	if (m_baseFile) fclose(m_baseFile); m_baseFile = NULL;
	if (m_outFile) fclose(m_outFile); m_outFile = NULL;
	if (m_logFile) fclose(m_logFile); m_logFile = NULL;
}

int MergeThread::readOneRtcm() {
	char c = 0;
	int stn = 0;
	bool is_complete = false;
	while (!feof(m_rtcmFile)) {
		if (m_isStop) break;
		fread(&c, sizeof(char), 1, m_rtcmFile);
		stn = input_aceinna_format_raw(c, m_outbuff, &m_outlen);
		if (TYPE_ROV == stn) {
			m_roverbuff.append((char*)m_outbuff, m_outlen);
			UpdateProcess(m_outlen);
		}
		else if (TYPE_BAS == stn) {
			m_basebuff.append((char*)m_outbuff, m_outlen);
			UpdateProcess(m_outlen);
		}
		if (TYPE_ROV == stn || TYPE_BAS == stn) {
			for (int j = 0; j < m_outlen; j++) {
				int ret = input_rtcm3(m_outbuff[j], stn - 1, &m_gnss);
				if (ret == 1) {
					is_complete = true;
					break;
				}
			}
		}
		if (is_complete) {
			break;
		}
	}
	return stn;
}

int MergeThread::readOneImu()
{
	char c = 0;
	char out_msg[1024] = { 0 };
	char imu_buffer[64] = { 0 };
	int ret = 0;
	if (m_imubuff.size() > 0) {
		fwrite(m_imubuff.data(), 1, m_imubuff.size(), m_outFile);
		m_imubuff.clear();
	}
	while (!feof(m_imuFile)) {
		if (m_isStop) break;
		fread(&c, sizeof(char), 1, m_imuFile);
		ret = input_imu_raw(c, out_msg);
		if (1 == ret) {
			user_s1_t* pak = getImuPak();
			if (pak->GPS_TimeOfWeek % 100 == 0 || (pak->GPS_TimeOfWeek / 10 > last_TimeOfWeek/10 && last_TimeOfWeek > 0)) {
				sprintf(imu_buffer, IMU_FLAG);
				memcpy(imu_buffer + ACEINNA_HEAD_SIZE, pak, sizeof(user_s1_t));
				m_imubuff.append(imu_buffer, IMU_CONST_SIZE + ACEINNA_HEAD_SIZE);
				UpdateProcess(ACEINNA_HEAD_SIZE + IMU_CONST_SIZE+2);
				fprintf(m_logFile, "$IMU,week:%d,sec:%d,len:%llu\n", pak->GPS_Week, pak->GPS_TimeOfWeek, sizeof(user_s1_t));
				break;
			}
			else {
				sprintf(imu_buffer, IMU_FLAG);
				memcpy(imu_buffer + ACEINNA_HEAD_SIZE, pak, sizeof(user_s1_t));
				fwrite(imu_buffer, 1, IMU_CONST_SIZE + ACEINNA_HEAD_SIZE, m_outFile);
				UpdateProcess(ACEINNA_HEAD_SIZE + IMU_CONST_SIZE + 2);
				fprintf(m_logFile, "$IMU,week:%d,sec:%d,len:%llu\n", pak->GPS_Week, pak->GPS_TimeOfWeek, sizeof(user_s1_t));
			}
			last_TimeOfWeek = pak->GPS_TimeOfWeek;
		}
	}
	return ret;
}

void MergeThread::readOneRover()
{
	char c = 0;
	int ret = 0;
	uint8_t* buffer = NULL;
	int len = 0;
	while (!feof(m_roverFile)) {
		fread(&c, sizeof(char), 1, m_roverFile);
		m_roverbuff.append(c);
		ret = input_rtcm3(c, 0, &m_gnss);
		if (is_complete_rtcm()) {
			UpdateProcess(m_roverbuff.size());
			fprintf(m_outFile, "%s1%03d", ROV_FLAG, m_roverbuff.size());
			fwrite(m_roverbuff.data(), 1, m_roverbuff.size(), m_outFile);
			//fprintf(m_logFile, "%s,1,%03d,%03d\n", ROV_FLAG, m_roverbuff.size(), m_roverbuff.size() + 8);
			buffer = (uint8_t*)m_roverbuff.data();
			len = m_roverbuff.size() - 3;
			fprintf(m_logFile, "%s,1,%03d, %d\n", ROV_FLAG, m_roverbuff.size(),rtcm_getbitu(buffer, len * 8, 24));
			m_roverbuff.clear();
		}
		if (ret == 1) {
			//fprintf(m_logFile, "rover,%f\n", m_gnss.rcv[0].time.time + m_gnss.rcv[0].time.sec);
			break;
		}
	}
}

void MergeThread::readOneBase()
{
	char c = 0;
	int ret = 0;
	uint8_t* buffer = NULL;
	int len = 0;
	if (m_gnss.rcv[1].time.time >= m_gnss.rcv[0].time.time) return;
	while (!feof(m_baseFile)) {
		fread(&c, sizeof(char), 1, m_baseFile);
		m_basebuff.append(c);
		ret = input_rtcm3(c, 1, &m_gnss);
		if (is_complete_rtcm()) {
			UpdateProcess(m_basebuff.size());
			fprintf(m_outFile, "%s%03d", BAS_FLAG, m_basebuff.size());
			fwrite(m_basebuff.data(), 1, m_basebuff.size(), m_outFile);
			//fprintf(m_logFile, "%s,0,%03d,%03d\n", BAS_FLAG, m_basebuff.size(), m_basebuff.size() + 8);
			buffer = (uint8_t*)m_basebuff.data();
			len = m_basebuff.size() - 3;
			fprintf(m_logFile, "%s,0,%03d, %d\n", BAS_FLAG, m_basebuff.size(), rtcm_getbitu(buffer, len * 8, 24));
			m_basebuff.clear();
		}
		if (ret == 1) {
			//fprintf(m_logFile, "base,%f\n", m_gnss.rcv[1].time.time + m_gnss.rcv[1].time.sec);
			break;
		}
	}
}

void MergeThread::UpdateProcess(int size) {
	m_RawFilesReadSize += size;
	double percent = (double)m_RawFilesReadSize / (double)m_RawFilesSize * 10000;
	emit sgnProgress((int)percent, m_TimeCounter.elapsed());
	//qDebug("%d/%d", m_RawFilesReadSize, m_RawFilesSize);
}

int MergeThread::getFileSize(FILE * file)
{
	fseek(file, 0L, SEEK_END);
	int file_size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	return file_size;
}