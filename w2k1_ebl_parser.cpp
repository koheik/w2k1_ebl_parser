// Copyright(C) 2018-2020 by Steven Adler
//
// This file is part of Actisense plugin for OpenCPN.
//
// Actisense plugin for OpenCPN is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Actisense plugin for OpenCPN is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the Actisense plugin for OpenCPN. If not, see <https://www.gnu.org/licenses/>.
//
// NMEA2000® is a registered trademark of the National Marine Electronics Association
// Actisense® is a registered trademark of Active Research Limited

// Project: Actisense Plugin
// Description: Actisense NGT-1 plugin for OpenCPN
// Unit: ActisenseEBL - Reads Actisense EBL Log Files
// Owner: twocanplugin@hotmail.com
// Date: 6/1/2020
// Version History: 
// 1.0 Initial Release
// #define DEBUG

#include <vector>
#include <fstream>
#include <iostream>
#include <glob.h>

typedef unsigned char byte;

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define CONVERT_MS_KNOTS 1.94384
#define CONVERT_MS_KMH 3.6
#define CONVERT_MS_MPH 2.23694
#define RADIANS_TO_DEGREES(x) (x * 180 / M_PI)

#define HEADING_TRUE 0
#define HEADING_MAGNETIC 1

// NMEA 183 GPS Fix Modes
#define GPS_MODE_AUTONOMOUS 'A' 
#define GPS_MODE_DIFFERENTIAL 'D' 
#define GPS_MODE_ESTIMATED 'E' 
#define GPS_MODE_MANUAL  'M'
#define GPS_MODE_SIMULATED 'S'
#define GPS_MODE_INVALID 'N' 

#define	WIND_REFERENCE_APPARENT 2

static bool IsDataValid(unsigned short value) {
    if ((value == USHRT_MAX) || (value == USHRT_MAX - 1) || (value == USHRT_MAX - 2)) {
        return FALSE;
    }
    else {
        return TRUE;
    }
}

// ASCII Control Chracters
const byte DLE = 0x10;
const byte STX = 0x02;
const byte ETX = 0x03;
const byte ESC = 0x1B;
const byte BEMSTART = 0x01;
const byte BEMEND = 0x0A;

static int ts_prev;
static int roll;

static bool isSynced;
static long offset;
static double cog;
static double sog;
static double awa;
static double aws;

void reset() {
    ts_prev = -1;
    roll = 0;

    isSynced = false;
    offset = 0;
    cog = 0.0;
    sog = 0.0;
    awa = 0.0;
    aws = 0.0;
}

// Decode PGN 126992 NMEA System Time
// $--ZDA, hhmmss.ss, xx, xx, xxxx, xx, xx*hh<CR><LF>
long DecodePGN126992(std::vector<byte> payload, std::vector<std::string> *nmeaSentences) {
    
    long unsigned int long ts = 0;
	if (payload.size() > 0) {

		byte sid;
		sid = payload[0];

		byte timeSource;
		timeSource = (payload[1] & 0xF) >> 4;

		unsigned short daysSinceEpoch;
		daysSinceEpoch = payload[2] | (payload[3] << 8);

		unsigned int secondsSinceMidnight = payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24);

        if (IsDataValid(daysSinceEpoch) && IsDataValid(secondsSinceMidnight)) {
            ts = 864000000L*(long unsigned int)daysSinceEpoch + (long unsigned int)secondsSinceMidnight;
            time_t t = ts / 10000;
            // std::cout << "System Time UTC: " << std::put_time(std::gmtime(&t), "%c %Z") << std::endl;
            isSynced = true;
        }
    }
    return ts;
}

bool DecodePGN129026(std::vector<byte> payload, double &cog, double &sog) {
	if (payload.size() > 0) {

		byte sid;
		sid = payload[0];

		// True = 0, Magnetic = 1
		byte headingReference;
		headingReference = (payload[1] & 0x03);

		unsigned short courseOverGround;
		courseOverGround = (payload[2] | (payload[3] << 8));

		unsigned short speedOverGround;
		speedOverGround = (payload[4] | (payload[5] << 8));

		// BUG BUG if Heading Ref = True (0), then ignore %.2f,M and vice versa if Heading Ref = Magnetic (1), ignore %.2f,T
		// BUG BUG GPS Mode should be obtained rather than assumed
		
        if (IsDataValid(courseOverGround) && IsDataValid(speedOverGround)) {
            cog = (float)courseOverGround / 10000;
            sog = (float)speedOverGround / 100;

#ifdef DEBUG
            printf("update: ref=%d, cog=%f, sog=%f\n", headingReference, RADIANS_TO_DEGREES(cog), sog);
#endif
            return TRUE;
        }
	}
    return FALSE;
}


// Decode PGN 130306 NMEA Wind
// $--MWV,x.x,a,x.x,a,A*hh<CR><LF>
bool DecodePGN130306(std::vector<byte> payload, double &awa, double &aws) {
	if (payload.size() > 0) {

		byte sid;
		sid = payload[0];

		unsigned short windSpeed;
		windSpeed = payload[1] | (payload[2] << 8);

		unsigned short windAngle;
		windAngle = payload[3] | (payload[4] << 8);

		byte windReference;
		windReference = (payload[5] & 0x07);

        if (IsDataValid(windAngle) && IsDataValid(windSpeed)) {
            awa = (float)windAngle / 10000;
            aws = (double)windSpeed / 100;

#ifdef DEBUG
            printf("update: ref=%d, angle=%f, speed=%f\n", windReference, RADIANS_TO_DEGREES(awa), aws);
#endif
            return TRUE;
        }
	}
    return FALSE;
}

void display(long int ts, char *buf, size_t n) {
    int h = ts / 1000 / 60 / 60;
    int m = (ts / 1000 / 60) - 60 * h;
    int s = (ts / 1000) - 60 * (60 * h + m);
    int ms = ts - 1000 * (ts / 1000);
    snprintf(buf, n, "%02d:%02d:%02d.%03d", h, m, s, ms);
}

void correction(double &cog, double &sog, double &awd, double &aws, double &twd, double &tws) {
    double vx = sog * std::sin(cog);
    double vy = sog * std::cos(cog);
    double ax = aws * std::sin(awd);
    double ay = aws * std::cos(awd);

    double tx = ax - vx;
    double ty = ay - vy;

    tws = std::sqrt(tx * tx + ty * ty);
    if (tws > 0) {
        if (tx >= 0 && ty >= 0) {
            twd = std::asin(tx / tws);
        } else if (tx >= 0 && ty < 0) {
            twd = M_PI - std::asin(tx / tws);
        } else if (tx < 0 && ty < 0) {
            twd = M_PI - std::asin(tx / tws);
        } else {
            twd = 2 * M_PI + std::asin(tx / tws);
        }
    } else {
        twd = 0;
    }
}

void process(std::vector<byte> assemblyBuffer) {
    std::vector<byte> payload;
    std::vector<std::string> nmeaSentences;

#ifdef DEBUGX
    for (int i = 0; i < assemblyBuffer.size(); i++) {
        printf(" %02X", assemblyBuffer.at(i));
    }
    printf("\n");
#endif

    if (assemblyBuffer.size() == 17) {
        long int ts = assemblyBuffer.at(3) + (assemblyBuffer.at(4) << 8);
        if (ts < ts_prev) {
            roll++;
        }
        ts_prev = ts;
        // printf("%04d %06ld\n", roll, ts);
        // if (isSynced) {
        //     time_t t = (offset + 10 * (roll * 65536L + ts)) / 10000;
        //     std::cout << "UTC: " << std::put_time(std::gmtime(&t), "%c %Z") << std::endl;
        // }
        int src = assemblyBuffer.at(5);
        int pgn = assemblyBuffer.at(6) 
            | (assemblyBuffer.at(7) << 8)
            | (assemblyBuffer.at(8) & 0x01) << 16;

        if (pgn == 126992) {
            char str[128];
            // display(ts + roll * 0xFFFF, str, 128);
            // std::cout << str << std::endl;
            // printf("%02d %05ld System Time\n", roll, ts);
            for (int i = 9; i < 17; i++)
                payload.push_back(assemblyBuffer.at(i));

            long system_time = DecodePGN126992(payload, &nmeaSentences);
            if (system_time > 0)
                offset = system_time - 10 * (65536L * roll + ts);
        }
        else if (pgn == 129026) {
#ifdef DEBUGX
            printf("%02d %05ld COG SOG\n", roll, ts);
#endif
            for (int i = 9; i < 17; i++)
                payload.push_back(assemblyBuffer.at(i));

            DecodePGN129026(payload, cog, sog);
        }
        else if (pgn == 130306) {
#ifdef DEBUGX
            printf("%02d %05ld Wind\n", roll, ts);
#endif

            for (int i = 9; i < 17; i++)
                payload.push_back(assemblyBuffer.at(i));

            if (isSynced && DecodePGN130306(payload, awa, aws)) {
                double twd, tws;
                double awd = awa + cog;
                while (awd > 2 * M_PI) {
                    awd -= 2 * M_PI;
                }
                correction(cog, sog, awd, aws, twd, tws);
#ifdef DEBUG
                printf("cog=%f, sog=%f\n", RADIANS_TO_DEGREES(cog), sog);
                printf("awa=%f, awd=%f, aws=%f\n", RADIANS_TO_DEGREES(awa), RADIANS_TO_DEGREES(awd), aws);
                printf("twd=%f, tws=%f\n", RADIANS_TO_DEGREES(twd), tws);
#endif
                time_t t = (offset + 10 * (roll * 65536L + ts)) / 10000;
                std::cout << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
                std::cout << ",";
                std::cout << RADIANS_TO_DEGREES(twd);
                std::cout << ",";
                std::cout << CONVERT_MS_KNOTS * tws;
                std::cout << std::endl;
            }
        }
    }
}

void read(std::string fname)
{
    // used to construct valid Actisense messages
	std::vector<byte> assemblyBuffer;
	// read 1K at a time
	std::vector<byte> readBuffer(1024,0);
	// used to iterate through the readBuffer
	std::streamsize bytesRead;
	// if we've found an ASCII Control Char DLE or ESC
	bool isEscaped = false;
	// if we've found an ASCII Control Char STX (preceded by a DLE)
	// or a BEMSTART (preceded by an ESC)
	bool msgStart = false;
	// if we've found an ASCII Control Char ETX (also preceded by a DLE)
	// or a BEMEND (preceded by an ESC)
	bool msgComplete = false;

    ts_prev = 0;
    roll = 0;
    std::ifstream logFileStream(fname, std::ios::binary);

    bytesRead = 1;
    while(bytesRead > 0) {
        logFileStream.read(reinterpret_cast<char *>(readBuffer.data()), readBuffer.size());
        bytesRead = logFileStream.gcount();

        for (int i=0; i < bytesRead; i++) {

            unsigned char ch = readBuffer.at(i);

#ifdef DEBUGX
            printf(" %02X |", ch);
            for (int j = 0; j < assemblyBuffer.size(); j++)
                printf(" %02X", assemblyBuffer.at(j));
            printf("\n");
#endif
            
            // if last character was DLE or ESC
            if (isEscaped) {
                isEscaped = false;

                // Message Start
                if ((ch == STX) && (!msgStart)) {
                    msgStart = true;
                    msgComplete = false;
                    assemblyBuffer.clear();
                }

                // Message End
                else if ((ch == ETX) && (msgStart)) {
                    msgComplete = true;
                    msgStart = false;
                }

                // Actisense Binary Encoded Message Start
                else if ((ch == BEMSTART) && (!msgStart)) {
                    msgStart = true;
                    msgComplete = false;
                    assemblyBuffer.clear();
                }

                // Actisense Binary Encoded Message End
                else if ((ch == BEMEND) && (msgStart)) {
                    msgComplete = true;
                    msgStart = false;
                }

                // Escaped DLE
                else if ((ch == DLE) && (msgStart)) {
                    assemblyBuffer.push_back(ch);
                }

                // Escaped ESC
                else if ((ch == ESC) && (msgStart)) {
                    assemblyBuffer.push_back(ch);
                }

                else {
                    // Can't have an escaped normal char
                    msgComplete = false;
                    msgStart = false;
                    assemblyBuffer.clear();
                }
            }
            // Previous character was not a DLE or ESC
            else {
                // if ((ch == DLE) || (ch == ESC)) {
                 if (ch == ESC) {
                    isEscaped = true;
                }
                else if (msgStart) {
                    // a normal character
                    assemblyBuffer.push_back(ch);
                }
            }
        
            if (msgComplete) {
                // we have a complete frame, process it
                process(assemblyBuffer);																					
                
                // Reset everything for next message
                assemblyBuffer.clear();
                msgStart = false;
                msgComplete = false;
                isEscaped = false;
                        
            }	// end if msgComplete
        
        } // end for
    } // end while
}

int main(int argc, char *argv[])
{

    if (argc != 2) {
        std::cerr << "Usage w2k1_elb_parser <data_dir>" << std::endl;
        return -1;
    }

    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));

    std::string pattern(argv[1]);
    pattern += "/*.ebl";
    int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if (return_value != 0) {
        globfree(&glob_result);
        std::cerr << "could not read directory: " << argv[1] << std::endl;
        return -1;
    }

    std::vector<std::string> filenames;
    for (size_t i = 0; i < glob_result.gl_pathc; i++) {
        filenames.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);

    std::cout << "ISODateTimeUTC,TWD,TWS" << std::endl;
    std::sort(filenames.begin(), filenames.end());
    for(size_t i = 0; i < filenames.size(); i++) {
        reset();
        read(filenames.at(i));
    }
    return 0;
}
