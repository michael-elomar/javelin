#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <map>
#include "javelin.hpp"
#include <vector>

namespace javelin {

// Get current timestamp as microseconds since epoch (8B)
uint64_t TelemetryLogger::getCurrentTimestamp()
{
	auto now = std::chrono::system_clock::now();
	auto duration = now.time_since_epoch();
	return std::chrono::duration_cast<std::chrono::microseconds>(duration)
		.count();
}

TelemetryLogger::TelemetryLogger()
{
	std::string filename = std::string(std::getenv("HOME")) + "/log.bin";
	mLogFile.open(filename, std::ios::binary | std::ios::trunc);
	if (!mLogFile.is_open()) {
		throw std::runtime_error("Failed to open binary log file: "
					 + filename);
	}
}

TelemetryLogger::TelemetryLogger(const std::string &filename)
	: mFilename(filename)
{
	mLogFile.open(mFilename, std::ios::binary | std::ios::trunc);
	if (!mLogFile.is_open()) {
		throw std::runtime_error("Failed to open binary log file: "
					 + mFilename);
	}
}

TelemetryLogger::~TelemetryLogger()
{
	if (mLogFile.is_open()) {
		mLogFile.close();
	}
}

// Log a single field (binary format)
void TelemetryLogger::log(const std::string &field, double value)
{
	std::lock_guard<std::mutex> lock(mLogMutex);

	// Write timestamp (8B)
	uint64_t timestamp = getCurrentTimestamp();
	mLogFile.write(reinterpret_cast<const char *>(&timestamp),
		       sizeof(timestamp));

	// Write field name (1B length + N B string)
	uint8_t fieldLength = static_cast<uint8_t>(field.size());
	mLogFile.write(reinterpret_cast<const char *>(&fieldLength),
		       sizeof(fieldLength));
	mLogFile.write(field.c_str(), fieldLength);

	// Write value (8B)
	mLogFile.write(reinterpret_cast<const char *>(&value), sizeof(value));

	mLogFile.flush();
}

// Log multiple fields at once
void TelemetryLogger::log(const std::map<std::string, double> &fields)
{
	std::lock_guard<std::mutex> lock(mLogMutex);
	uint64_t timestamp = getCurrentTimestamp();

	for (const auto &[field, value] : fields) {
		// Write timestamp for each field (or use a single
		// timestamp for all)
		mLogFile.write(reinterpret_cast<const char *>(&timestamp),
			       sizeof(timestamp));

		// Write field name
		uint8_t fieldLength = static_cast<uint8_t>(field.size());
		mLogFile.write(reinterpret_cast<const char *>(&fieldLength),
			       sizeof(fieldLength));
		mLogFile.write(field.c_str(), fieldLength);

		// Write value
		mLogFile.write(reinterpret_cast<const char *>(&value),
			       sizeof(value));
	}
	mLogFile.flush();
}

TelemetryReader::TelemetryReader(const std::string &filename)
	: filename(filename)
{
	logFile.open(filename, std::ios::binary | std::ios::in);
	if (!logFile.is_open()) {
		throw std::runtime_error("Failed to open binary log file: "
					 + filename);
	}
}

TelemetryReader::~TelemetryReader()
{
	if (logFile.is_open()) {
		logFile.close();
	}
}

std::vector<TelemetryEntry> TelemetryReader::readAll()
{
	std::vector<TelemetryEntry> entries;
	TelemetryEntry entry;

	while (readNext(entry)) {
		entries.push_back(entry);
	}

	return entries;
}

bool TelemetryReader::readNext(TelemetryEntry &entry)
{
	return readEntry(entry);
}

void TelemetryReader::reset()
{
	logFile.clear();                 // Clear any error flags
	logFile.seekg(0, std::ios::beg); // Seek to the beginning
}

bool TelemetryReader::readEntry(TelemetryEntry &entry)
{
	// Read timestamp (8B)
	if (!logFile.read(reinterpret_cast<char *>(&entry.timestamp),
			  sizeof(entry.timestamp))) {
		return false; // End of file or error
	}

	// Read field name length (1B)
	uint8_t fieldLength;
	if (!logFile.read(reinterpret_cast<char *>(&fieldLength),
			  sizeof(fieldLength))) {
		return false;
	}

	// Read field name (N B)
	entry.field.resize(fieldLength);
	if (!logFile.read(&entry.field[0], fieldLength)) {
		return false;
	}

	// Read value (8B)
	if (!logFile.read(reinterpret_cast<char *>(&entry.value),
			  sizeof(entry.value))) {
		return false;
	}

	return true;
}

} // namespace javelin
