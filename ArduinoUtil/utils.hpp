#ifndef utils_hpp
#define utils_hpp

#ifndef Arduino_h
#include<Arduino.h>
#endif

class SerialHandler {
  public :
    SerialHandler(usb_serial_class& device) {
      hwStream = &device;
    }
    float fetchValue()
    {
      int i = 0;
      char string[15];

      while (i < (int)sizeof(string))
      {
        if (hwStream->available())
        {
          char c = hwStream->read();
          if ((c >= '0' && c <= '9') || c == '-' || c == '.')
          {
            string[i] = c;
            i++;
          }
          else
          {
            string[i] = '\0';
            break;
          }
        }
      }
      return atof(string);
    }
    int Printf(char *str, ...)
    {
      int i, count=0, j=0;
      char temp[128+1];
      for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

      va_list argv;
      va_start(argv, count);
      for(i=0,j=0; str[i]!='\0';i++)
      {
        if(str[i]=='%')
        {
          temp[j] = '\0';
          hwStream->print(temp);
          j=0;
          temp[0] = '\0';

          switch(str[++i])
          {
            case 'd': hwStream->print(va_arg(argv, int));
                      break;
            case 'l': hwStream->print(va_arg(argv, long));
                      break;
            case 'f': hwStream->print(va_arg(argv, double),5);
                      break;
            case 'c': hwStream->print((char)va_arg(argv, int));
                      break;
            case 's': hwStream->print(va_arg(argv, char *));
                      break;
            default:  ;
          };
        }
        else 
        {
          temp[j] = str[i];
          
          j = (j+1)%128;
          if(j==0) 
          {
            temp[128] = '\0';
            hwStream->print(temp);
            temp[0]='\0';
          }
        }
        Serial.flush();
      };
      hwStream->println();
      Serial.flush();
      return count + 1;
    }
    usb_serial_class* hwStream;
    
};

#endif