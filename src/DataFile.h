/*
 *  DataFile.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// DataFile.h - utility class to read in various sorts of data files

#ifndef DataFile_h
#define DataFile_h

#include <stddef.h>

//{

class DataFile
{
public:
    DataFile();
    ~DataFile();

    // initialise by reading the file
    // this routine allocates a buffer and reads in the whole file
    bool ReadFile(const char * const name);
    // write the data out to a file
    bool WriteFile(const char * const name, bool binary = false);

#ifdef _WIN32
    // provide Windows specific wchar versions
    bool ReadFile(const wchar_t * const name);
    bool WriteFile(const wchar_t * const name, bool binary = false);
#endif

    // retrieve basic types
    // the file is searched for the parameter and then the next token is read
    bool RetrieveParameter(const char * const param,
                           int *val, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param,
                           long *val, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param,
                           unsigned int *val, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param,
                           unsigned long *val, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param,
                           double *val, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param,
                           bool *val, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param,
                           char *val, long size, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param,
                           char **val, ptrdiff_t *size, bool searchFromStart = true);
    bool RetrieveQuotedStringParameter(const char * const param,
                                       char *val, long size, bool searchFromStart = true);
    bool RetrieveQuotedStringParameter(const char * const param, char **val,
                                       ptrdiff_t *size, bool searchFromStart = true);

    // ranged functions
    // the file is searched for the parameter and then the next token is read
    bool RetrieveRangedParameter(const char * const param,
                                 double *val, bool searchFromStart = true);
    void SetRangeControl(double r) { m_RangeControl = r; }

    // line reading functions
    // optional comment character and can ignore empty lines
    bool ReadNextLine(char *line, long size, bool ignoreEmpty,
                      char commentChar = 0, char continuationChar = 0);
    bool ReadNextLine2(char *line, long size, bool ignoreEmpty,
                       const char *commentString, const char *continuationString = 0);

    // retrieve arrays
    // the file is searched for the parameter and then the next token is read
    bool RetrieveParameter(const char * const param, long n,
                           int *val, bool searchFromStart = true);
    bool RetrieveParameter(const char * const param, long n,
                           double *val, bool searchFromStart = true);
    bool RetrieveRangedParameter(const char * const param, long n,
                                 double *val, bool searchFromStart = true);

    // utility settings
    void SetExitOnError(bool flag) { m_ExitOnErrorFlag = flag; }
    bool GetExitOnError() { return m_ExitOnErrorFlag; }
    char *GetRawData() { return m_FileData; }
    void SetRawData(const char *string);
    void ResetIndex() { m_Index = m_FileData; }
    size_t GetSize() { return m_Size; }
    void ClearData();
    long Replace(const char *oldString, const char *newString);
    char *GetIndex() { return m_Index; }
    void SetIndex(char *index) { m_Index = index; }
    char *GetPathName() { return m_PathName; }
#ifdef _WIN32
    wchar_t *GetWPathName() { return m_WPathName; }
#endif

    // probably mostly for internal use
    // read the next ASCII token from the current index and bump index
    bool FindParameter(const char * const param, bool searchFromStart = true);
    bool ReadNext(int *val);
    bool ReadNext(double *val);
    bool ReadNext(char *val, long size);
    bool ReadNext(char **val, ptrdiff_t *size);
    bool ReadNextQuotedString(char *val, long size);
    bool ReadNextQuotedString(char **val, ptrdiff_t *size);
    bool ReadNextLine(char *line, long size);
    bool ReadNextRanged(double *val);

    // handy statics
    static bool EndOfLineTest(char **p);
    static long CountTokens(const char *string);
    static long ReturnTokens(char *string, char *ptrs[], long size);
    static long CountTokens(const char *string, const char *separators);
    static long ReturnTokens(char *string, char *ptrs[], long size, const char *separators);
    static long CountLines(const char *string);
    static long ReturnLines(char *string, char *ptrs[], long size);
    static void Strip(char *str);
    static bool StringEndsWith(const char * str, const char * suffix);
    static bool StringStartsWith(const char * str, const char * suffix);
    static double Double(const char *buf);
    static void Double(const char *buf, long n, double *d);
    static int Int(const char *buf);
    static void Int(const char *buf, long n, int *d);
    static bool Bool(const char *buf);
    static bool EndsWith(const char *str, const char *suffix);
#ifdef _WIN32
    static char *ConvertWideToUTF8(const wchar_t *input);
    static wchar_t *ConvertUTF8ToWide(const char *input);
#endif

    // binary file operators
    // read the next binary value from current index and bump index
    bool ReadNextBinary(int *val);
    bool ReadNextBinary(float *val);
    bool ReadNextBinary(double *val);
    bool ReadNextBinary(char *val);
    bool ReadNextBinary(bool *val);
    bool ReadNextBinary(int *val, long n);
    bool ReadNextBinary(float *val, long n);
    bool ReadNextBinary(double *val, long n);
    bool ReadNextBinary(char *val, long n);
    bool ReadNextBinary(bool *val, long n);

    // file writing operators
    bool WriteParameter(const char * const param, int val);
    bool WriteParameter(const char * const param, long val);
    bool WriteParameter(const char * const param, double val);
    bool WriteParameter(const char * const param, bool val);
    bool WriteParameter(const char * const param, const char * const val);
    bool WriteQuotedStringParameter(const char * const param, const char * const val);
    bool WriteParameter(const char * const param, long n, int *val);
    bool WriteParameter(const char * const param, long n, long *val);
    bool WriteParameter(const char * const param, long n, double *val);
    bool WriteParameter(const char * const param, long n, bool *val);
    bool WriteNext(int val, char after = 0);
    bool WriteNext(long val, char after = 0);
    bool WriteNext(double val, char after = 0);
    bool WriteNext(bool val, char after = 0);
    bool WriteNext(const char * const val, char after = 0);
    bool WriteNextQuotedString(const char * const val, char after = 0);

protected:

    char * m_FileData;
    char * m_Index;
    bool m_ExitOnErrorFlag;
    double m_RangeControl;
    ptrdiff_t m_Size;
    char * m_PathName;
#ifdef _WIN32
    wchar_t * m_WPathName;
#endif
};

//}

#endif

