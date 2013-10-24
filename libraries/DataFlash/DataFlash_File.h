/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_File_h
#define DataFlash_File_h

class DataFlash_File : public DataFlash_Class
{
public:
    // constructor
    DataFlash_File(const char *log_directory);

    // initialisation
    void Init(void);
    bool CardInserted(void);

    // erase handling
    bool NeedErase(void);
    void EraseAll();

    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);

    // high level interface
    uint16_t find_last_log(void);
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    uint16_t get_num_logs(void);
    uint16_t start_new_log(void);
    void LogReadProcess(uint16_t log_num,
                        uint16_t start_page, uint16_t end_page, 
                        uint8_t num_types,
                        const struct LogStructure *structure,
                        void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                        AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);

private:
    int _write_fd;
    int _read_fd;
    uint32_t _read_offset;
    volatile bool _initialised;
    const char *_log_directory;

    /*
      read a block
    */
    void ReadBlock(void *pkt, uint16_t size);

    // write buffer
    uint8_t *_writebuf;
    const uint16_t _writebuf_size;
    volatile uint16_t _writebuf_head;
    volatile uint16_t _writebuf_tail;
    uint32_t _last_write_time;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(uint16_t log_num);
    char *_lastlog_file_name(void);
    uint32_t _get_log_size(uint16_t log_num);

    void _io_timer(void);
};


#endif // DataFlash_File_h

