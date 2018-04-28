/*
 *  XMLConverter.h
 *  GA
 *
 *  Created by Bill Sellers on Fri Dec 12 2003.
 *  Copyright (c) 2003 Bill Sellers. All rights reserved.
 *
 */

#ifndef XMLConverter_h
#define XMLConverter_h

#include <vector>
#include <string>

class Genome;
class DataFile;
class ExpressionParser;

class XMLConverter
{
public:
    XMLConverter();
    ~XMLConverter();

    int LoadBaseXMLFile(const char *filename);
    int LoadBaseXMLString(char *dataPtr);
    int ApplyGenome(int genomeSize, double *genomeData);
    char* GetFormattedXML(int * docTxtLen);
    bool GetSmartSubstitutionFlag() { return m_SmartSubstitutionFlag; }
    void SetSmartSubstitutionFlag(bool smartSubstitutionFlag) { m_SmartSubstitutionFlag = smartSubstitutionFlag; }

protected:

    void ConvertVectorBrackets();

    bool m_SmartSubstitutionFlag;

    std::vector<std::string *> m_SmartSubstitutionTextComponents;
    std::vector<std::string *> m_SmartSubstitutionParserText;
    std::vector<double> m_SmartSubstitutionValues;
    char *m_SmartSubstitutionTextBuffer;
    int m_DocTxtLen;
};


#endif
