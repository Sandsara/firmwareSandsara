#pragma once

#include "config.h"
//#define FS_NO_GLOBALS
//#include <FS.h>
#include <SPI.h>
#include "SdFat.h"


/**
 * @class SdFiles
 * @brief Se encarga de gestionar la lectura de la memoria SD.
 * @param fileName contiene el nombre del archivo que se va a leer.
 * @param pFile se utiliza como apuntador a alguna linea del archivo.
 * @param fileType almacena el tipo de archivo pudiendo ser:
 * - 1 --> .txt
 * - 2 --> .thr
 * - 3 --> .bin
 * - -1 --> ninguno de los anteriores
 * @param directionMode almacena la direccion de lectura del archivo, pudiendo ser:
 * - 0 --> lo leera de abajo hacia arriba (en reversa)
 * - 1 --> lo leera de arriba hacia abajo (ejecucion normal)
 * @param file es un objeto de tipo File para el uso de la SD.
 * @param pFileBin se utiliza como apuntador a la linea de un archivo .bin.
 * @param charsToRead almacena el numero de bytes que se deben leer.
 * @param statusFile almacena un codigo de error que puede significar lo siguiente
 * - 0 no hubo problemas
 * - 1 ya no hay mas lineas por leer en el archivo
 * - -1 no se encontro el separador de los componentes (',' o ' ')
 * - -4 no se encontro un segundo componente
 * - -6 no es un tipo de archivo valido
 * @param componentSeparator se utiliza para identificar el caracter que separa los 2 componentes del archivo.
 * @param lineSeparator se utiliza para identificar el caracter que separa a cada linea del archivo.
 * @param dataBufferBin se utiliza para almacenar los datos de la SD.
 * @param currentRow almacena una linea del archivo como texto.
 * @param dataBuffer almacena parte del archivo de la SD.
 */
class SdFiles {
    public:
        String fileName;
        long pFile;
        int fileType;//1 for txt 2 for thr and 3 for bin
        int directionMode; //1 for fordward and 0 for backwards
        File file;
        static int DISTANCIA_MAX; //distance in millimeters
    private:
        int16_t xbin, ybin;
        int pFileBin;
        int charsToRead = 1000;
        int statusFile = 0;
        double z, theta;
        char componentSeparator = ',';
        char lineSeparator = '\n';
        uint8_t* dataBufferBin;
        String currentRow = "";
        String dataBuffer = "";
        int fileSize;
        bool noMoreData = false, lastRow = false;        
    public:
        SdFiles(String , int = 1);
        ~SdFiles();
        int getNextComponents(double* , double* , bool = true);
        int getStatus();
        void autoSetMode(double );
        double getStartPoint(int = 1, int = 0);
        double getFinalPoint(int = 1, int = 0);
        double getFinalAngle();
        double getStartAngle();
        double getStartModule();
        static int creatListOfFiles(String );
        static int getLineNumber(int , String , String& );
        static int numberOfLines(String );
        bool isValid();
        static int getType(String);
    private:
        int getComponents(String , double* , double*);
        int getComponentsBin(uint8_t* , double* , double* );
        String nextRow();
        int readFile();
};
