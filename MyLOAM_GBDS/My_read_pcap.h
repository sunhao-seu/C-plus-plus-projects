#pragma once
#ifndef VLP16_READ_H
#define VLP16_READ_H

#include "winpcap/pcap/pcap.h"
#include<iostream>
#include<vector>
const int HDL_NUM_ROT_ANGLES = 36001;
const int HDL_LASER_PER_FIRING = 32;
const int HDL_MAX_NUM_LASERS = 64;
const int HDL_FIRING_PER_PKT = 12;
typedef long long vtkIdType;

enum HDLBlock
{
	BLOCK_0_TO_31 = 0xeeff,
	BLOCK_32_TO_63 = 0xddff
};
#pragma pack(push, 1)
typedef struct HDLLaserReturn
{
	unsigned short distance;
	unsigned char intensity;
} HDLLaserReturn;
struct HDLFiringData
{
	unsigned short blockIdentifier;
	unsigned short rotationalPosition;
	HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};
struct HDLDataPacket
{
	HDLFiringData firingData[HDL_FIRING_PER_PKT];
	unsigned int gpsTimestamp;
	unsigned char blank1;
	unsigned char blank2;
};
struct HDLLaserCorrection
{
	double azimuthCorrection;
	double verticalCorrection;
	double distanceCorrection;
	double verticalOffsetCorrection;
	double horizontalOffsetCorrection;
	double sinVertCorrection;
	double cosVertCorrection;
	double sinVertOffsetCorrection;
	double cosVertOffsetCorrection;
};
struct HDLRGB
{
	unsigned char r;
	unsigned char g;
	unsigned char b;
};
#pragma pack(pop)
struct _array
{
	double xyz[3];
	double Intensity;
	int  LaserId;
	double Distance;
};
// Some versions of libpcap do not have PCAP_NETMASK_UNKNOWN
#if !defined(PCAP_NETMASK_UNKNOWN)
#define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif
class vtkPacketFileReader
{
public:

	vtkPacketFileReader()
	{
		this->PCAPFile = 0;
	}

	~vtkPacketFileReader()
	{
		this->Close();
	}

	bool Open(const std::string& filename)
	{
		char errbuff[PCAP_ERRBUF_SIZE];
		pcap_t *pcapFile = pcap_open_offline(filename.c_str(), errbuff);
		if (!pcapFile)
		{
			this->LastError = errbuff;
			return false;
		}

		struct bpf_program filter;

		if (pcap_compile(pcapFile, &filter, "udp", 0, PCAP_NETMASK_UNKNOWN) == -1)
		{
			this->LastError = pcap_geterr(pcapFile);
			return false;
		}

		if (pcap_setfilter(pcapFile, &filter) == -1)
		{
			this->LastError = pcap_geterr(pcapFile);
			return false;
		}

		this->FileName = filename;
		this->PCAPFile = pcapFile;
		this->StartTime.tv_sec = this->StartTime.tv_usec = 0;
		return true;
	}

	bool IsOpen()
	{
		return (this->PCAPFile != 0);
	}

	void Close()
	{
		if (this->PCAPFile)
		{
			pcap_close(this->PCAPFile);
			this->PCAPFile = 0;
			this->FileName.clear();
		}
	}

	const std::string& GetLastError()
	{
		return this->LastError;
	}

	const std::string& GetFileName()
	{
		return this->FileName;
	}

	void GetFilePosition(fpos_t* position)
	{
#ifdef _MSC_VER
		pcap_fgetpos(this->PCAPFile, position);
#else
		FILE* f = pcap_file(this->PCAPFile);
		fgetpos(f, position);
#endif
	}

	void SetFilePosition(fpos_t* position)
	{
#ifdef _MSC_VER
		pcap_fsetpos(this->PCAPFile, position);
#else
		FILE* f = pcap_file(this->PCAPFile);
		fsetpos(f, position);
#endif
	}

	bool NextPacket(const unsigned char*& data, unsigned int& dataLength, double& timeSinceStart, pcap_pkthdr** headerReference = NULL)
	{
		if (!this->PCAPFile)
		{
			return false;
		}

		struct pcap_pkthdr *header;
		int returnValue = pcap_next_ex(this->PCAPFile, &header, &data);
		if (returnValue < 0)
		{
			this->Close();
			return false;
		}

		if (headerReference != NULL)
		{
			*headerReference = header;
			dataLength = header->len;
			timeSinceStart = GetElapsedTime(header->ts, this->StartTime);
			return true;
		}

		// The ethernet header is 42 bytes long; unnecessary
		/*const unsigned int bytesToSkip = 42;
		dataLength = header->len - bytesToSkip;*/
		const unsigned int bytesToSkip = 42;
		if (header->len == 1248) {
			dataLength = header->len - bytesToSkip;
		}
		else {
			dataLength = header->len - bytesToSkip - 2;
		}

		data = data + bytesToSkip;
		timeSinceStart = GetElapsedTime(header->ts, this->StartTime);

		return true;
	}

protected:

	double GetElapsedTime(const struct timeval& end, const struct timeval& start)
	{
		return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.00;
	}

	pcap_t* PCAPFile;
	std::string FileName;
	std::string LastError;
	struct timeval StartTime;
};
class MyHdlGrabber
{
public:
	MyHdlGrabber(const std::string& filename, const std::string& correctionsFile);
	~MyHdlGrabber();
public:
	bool GetFrame(int frameNumber);
	int ReadFrameInformation(void);
	int GetNumberOfFrames(void);
	void SetCorrectionsFile(const std::string& correctionsFile);
	void LoadCorrectionsFile(const std::string& correctionsFile);
	void SetFileName(const std::string& filename);
	void Init(const std::string& filename, const std::string& correctionsFile);
	void Close();
	void Open();
private:
	void InitTables();
	void ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived);
	void SplitFrame(bool force);
	void ProcessFiring(HDLFiringData* firingData,
		int hdl64offset,
		int firingBlock,
		int azimuthDiff,
		double timestamp,
		unsigned int rawtime);
	void PushFiringData(const unsigned char laserId,
		const unsigned char rawLaserId,
		unsigned short azimuth,
		const double timestamp,
		const unsigned int rawtime,
		const HDLLaserReturn* laserReturn,
		const HDLLaserCorrection* correction,
		const bool dualReturn);
	double VLP16AdjustTimeStamp(int firingblock,
		int dsr,
		int firingwithinblock);
	double HDL32AdjustTimeStamp(int firingblock,
		int dsr);
	void UnloadData();

public:
	std::vector<_array>Points;

protected:
	std::vector<fpos_t> FilePositions;
	std::vector<int> Skips;


	int CalibrationReportedNumLasers;
	bool CorrectionsInitialized;

	// User configurable parameters
	int NumberOfTrailingFrames;
	int ApplyTransform;
	int PointsSkip;
	int SplitCounter;

	bool CropReturns;
	bool CropInside;
	double CropRegion[6];
	std::vector<double> cos_lookup_table_;
	std::vector<double> sin_lookup_table_;
	std::vector<bool> LaserSelection;
	unsigned int DualReturnFilter;

	std::string CorrectionsFile;
	std::string FileName;

	int Skip;
	bool endFrame = 0;
};
#endif