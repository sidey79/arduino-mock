#ifndef ARDUINO_MOCK_MOCKSTREAM_H
#define ARDUINO_MOCK_MOCKSTREAM_H

#include <Stream.h>
#include <Print.h>
#include <vector>
#include <cstdint>
#include <cstring>

namespace arduino_mock {

class MockStream : public Stream {
public:
    MockStream() : write_count_(0) {}

    // --- Verification Methods ---
    const std::vector<uint8_t>& getWrittenBytes() const {
        return written_data_;
    }

    size_t getWriteCount() const {
        return write_count_;
    }

    void clearData() {
        written_data_.clear();
        write_count_ = 0;
    }

    // --- Stream/Print Overrides ---

    // Required by Print (and thus Stream)
    size_t write(uint8_t byte) {
        written_data_.push_back(byte);
        write_count_++;
        return 1;
    }

    // Required by Stream
    size_t write(const uint8_t *buffer, size_t size) override {
        if (buffer == nullptr || size == 0) {
            return 0;
        }
        written_data_.insert(written_data_.end(), buffer, buffer + size);
        write_count_ += size;
        return size;
    }

    // Required by Stream (Pure Virtual)
    int available() override {
        return 0; // Mock: Nothing available to read
    }

    // Required by Stream (Pure Virtual)
    int read() override {
        return -1; // Mock: Nothing to read
    }

    // Required by Stream (Pure Virtual)
    int peek() override {
        return -1; // Mock: Nothing to peek
    }

    // Required by Stream (Pure Virtual)
    void flush() override {
        // Mock: Do nothing
    }

    // --- Other Stream Methods (Pure Virtual in Stream, must be overridden) ---

    size_t readBytes( char *buffer, size_t length) override { return 0; }
    size_t readBytes( uint8_t *buffer, size_t length) override { return 0; }
    bool find(char *target, size_t length) override { return false; }
    bool find(uint8_t *target, size_t length) override { return false; }
    
    // Implement remaining pure virtual methods without 'override' and avoid String usage.
    size_t readStringUntil(char terminator) { return 0; }
    size_t readString() { return 0; }
    size_t readString(size_t length) { return 0; }


private:
    std::vector<uint8_t> written_data_;
    size_t write_count_;
};

} // namespace arduino_mock

#endif // ARDUINO_MOCK_MOCKSTREAM_H