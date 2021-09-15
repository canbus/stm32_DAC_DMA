#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#pragma pack(1)
//wave 文件格式参考https://www.cnblogs.com/ranson7zop/p/7657874.html
typedef struct 
{
    uint8_t chunkid[4]; //RIFF
    uint32_t chunkSize; //文件剩下的总字节数
    uint8_t format[4];  //WAVE
    
    uint8_t subChunkid[4];//fmt
    uint32_t subChunkSize;
    uint16_t audioFormat;
    uint16_t numChns;    //通道数
    uint32_t sampleRate; 
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPreSample;
}Head_Wav;
#pragma pack()

void main(int argc,char **argv)
{
    if(argc < 2){
        printf("Usage:%s sample.wav",argv[0]);
        return;
    }
    FILE *in = fopen(argv[1],"rb");

    uint8_t outFile[100];
    sprintf(outFile,"%s.h",argv[1]);
    FILE *out = fopen(outFile,"w");
    assert(in);
    assert(out);
    
    Head_Wav headWave;
    fread(&headWave,sizeof(headWave),1,in);
    printf("format:%c%c%c%c,%c%c%c%c\n",headWave.chunkid[0],headWave.chunkid[1],headWave.chunkid[2],headWave.chunkid[3],
        headWave.format[0],headWave.format[2],headWave.format[3],headWave.format[4]);
    printf("chn:%d,sampleRate:%d,bitsPreSample:%d\n", headWave.numChns,headWave.sampleRate,headWave.bitsPreSample);
    if(strncmp(headWave.chunkid,"RIFF",4) != 0 || strncmp(headWave.format,"WAVE",4) != 0
      || headWave.numChns != 1 ){
        printf("unsupport format!");
        if(headWave.numChns != 1)
          printf("only support single channel\n");
        fclose(in);
        fclose(out);
        return;
    }
    int headWaveSize  = sizeof(headWave);
    if(headWave.subChunkSize > 16)
        headWaveSize += headWave.subChunkSize - 16; //多出来的填充空节
    uint8_t subChunkid2[5] = {0};
    uint32_t subChunkSize2 = 0;
    fseek(in,headWaveSize,0);
    fread(&subChunkid2,4,1,in);
    fread(&subChunkSize2,4,1,in);

    printf("subChunkid2:%s,size:%d\n",subChunkid2,subChunkSize2);
    
    int16_t val,i=0;
    fwrite("uint16_t audio_dat[] = {\r\n",26,1,out);
    for(i=1;i< (subChunkSize2 / 2);i++){
      if (fread(&val, sizeof(int16_t), 1, in) == 1) {
        val = val / 10;  //缩小10倍
        uint16_t uVal = val + 0x0FFF / 2;
        // uint16_t uVal = val > 0 ? val / 3 : 0;
        if (uVal >= 0x0FFF) uVal = 0x0FFF;
        uint8_t hexVal[10];
        sprintf(hexVal, "0x%04X,", uVal);
        printf("%d:%d,%d,%s %.2fV\n", i, val, uVal, hexVal,
               uVal * 3.3 / 0x0FFF);
        fwrite(hexVal, 7, 1, out);
        if (i % 10 == 0 ) {
          fwrite("\r\n", 2, 1, out);
          //break;
        }
      }
    }
    fwrite("};",2,1,out);
    printf("save to:%s\n",outFile);

    printf("format:%c%c%c%c,%c%c%c%c\n",headWave.chunkid[0],headWave.chunkid[1],headWave.chunkid[2],headWave.chunkid[3],
        headWave.format[0],headWave.format[2],headWave.format[3],headWave.format[4]);
    printf("chn:%d,sampleRate:%d,bitsPreSample:%d\n", headWave.numChns,headWave.sampleRate,headWave.bitsPreSample);
    
    fclose(in);
    fclose(out);
}