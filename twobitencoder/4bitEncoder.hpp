#include<Arduino.h>
#include<Encoder.h>

class FourBitEncoder {
    public:
        FourBitEncoder(){
        }
        int read() {
            return count_;
        }
        void write(int count) {
            count_ = count;
        }
        void setReverse(bool isReversed) {
            isReversed_ = isReversed;
        }
        void update(uint8_t sig) {
            if (sig == old_sig_) return;
            if (
                (sig == B00 && old_sig_ == B01) ||
                (sig == B01 && old_sig_ == B11) ||
                (sig == B11 && old_sig_ == B10) ||
                (sig == B10 && old_sig_ == B00)
            ) {
                if (!isReversed_) {
                    count_++;
                } else {
                    count_--;
                }
            }
            else if (
                (sig == B00 && old_sig_ == B10) ||
                (sig == B10 && old_sig_ == B11) ||
                (sig == B11 && old_sig_ == B01) ||
                (sig == B01 && old_sig_ == B00)
            ) {
                if (!isReversed_) {
                    count_--;
                } else {
                    count_++;
                }         
            }
            else {
                Serial.print("[CAUTION ]Skip reading occured!! ");
                Serial.print("old : ");
                Serial.print(old_sig_);
                Serial.print(" now : ");
                Serial.print(sig);
                Serial.println();
            }
            old_sig_ = sig;
        }
    
    private :
        int count_=0;
        int old_sig_;
        bool isReversed_ = false;  
};