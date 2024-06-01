#include <iostream>
#include <string>
#include <vector>

#include "I2CTransactionBuilder.hpp"

using namespace std; 

int main()
{
    
    int fd = I2CTransactionHelper::openDevice("/dev/i2c-7"); 
    if(fd < 0)
    {
        cout << "Error: Unable to open I2C device. Terminating." << endl; 
        return 0; 
    }

    cout << "Please enter device address: "; 
    int devaddr; 
    cin >> devaddr; 
    cout << endl; 
    if(devaddr < 0 || devaddr > 0x7F)
    {
        cout << "Error: Device address invalid." << endl; 
        return 0; 
    }

    cout << "Please enter transaction type. 1: WR, 2: RD, 3: WR+RD: "; 
    int type; 
    cin >> type; 
    cout << endl; 
    if(type < 1 || type > 3)
    {
        cout << "Error: Transaction type invalid." << endl; 
        return 0; 
    }

    I2CTransactionHelper transaction {devaddr}; 

    vector<uint8_t> wrData; 
    vector<uint8_t> rdData; 

    if(type & 0x1) // Handle write
    {
        cout << "Enter bytes to write." << endl << "> "; 
        string hexstr; 
        cin >> hexstr; 
        cout << endl; 
        // Parse string
        bool readingMsb = true; // start of number
        uint8_t temp = 0; 
        for(char ch : hexstr)
        {
            bool valid = false; 
            uint8_t num = 0; 
            if(ch >= '0' && ch <= '9')
            {
                valid = true; 
                num = ch - '0'; 
            }
            if(ch >= 'a' && ch <= 'f')
            {
                valid = true; 
                num = ch - 'a' + 10; 
            }
            if(ch >= 'A' && ch <= 'F')
            {
                valid = true; 
                num = ch - 'A' + 10; 
            }
            if(readingMsb)
            {
                if(!valid)
                {
                    continue; 
                }
                temp = num; 
                readingMsb = false; 
            }
            else
            {
                if(!valid)
                {
                    num = temp; 
                }
                else
                {
                    num = num | temp << 4; 
                }
                wrData.push_back(num); 
                readingMsb = true; 
            }
        }
        if(wrData.size() == 0)
        {
            cout << "Error: Nothing to write." << endl; 
            return 0; 
        }
        // Load segment
        transaction.writeVector(wrData); 
    }
    if(type & 0x2) // Handle read
    {
        int size; 
        cout << "Enter bytes to read: "; 
        cin >> size; 
        cout << endl; 
        if(size <= 0)
        {
            cout << "Error: Nothing to read." << endl; 
            return 0; 
        }
        rdData.resize(size); 
        // Load segment
        transaction.readVector(rdData); 
    }

    // Transaction preview
    cout << "Transaction preview: " << endl; 
    if(type & 0x1)
    {
        cout << "S ADDR(W) " << hex << setw(2); 
        for(uint8_t d : wrData)
        {
            cout << d << " "; 
        }
    }
    if(type & 0x2)
    {
        cout << "S ADDR(R) "; 
        for(uint8_t ignored : rdData)
        {
            cout << "XX "; 
        }
    }
    cout << "P" << endl; 

    cout << "Press any key to continue."; 
    std::cin.ignore(1000000, '\n');

    // Perform transaction
    transaction.doTransaction(fd); 
    cout << "Transaction performed." << endl; 
    if(type & 0x2)
    {
        cout << "Data read: " << hex << setw(2); 
        for(uint8_t d : rdData)
        {
            cout << d << " "; 
        }
        cout << endl; 
    }

    I2CTransactionHelper::closeDevice(fd); 

    return 0; 
}
