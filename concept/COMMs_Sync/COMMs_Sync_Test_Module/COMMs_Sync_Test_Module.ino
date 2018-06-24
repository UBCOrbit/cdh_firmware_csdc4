/*
 * Testing CDH's ability to receive UART communication from COMMS
 * Generates packet of at most 256 bytes, uses UART communications to transmit it to STM
 */

//elements of packet structure
/*form the first byte, made to be changed during testing*/
#define START 0b0110
#define ADDRESS 0b000
#define DATA_FLAG 0b1
/*forms the second byte, arbitrarily chosen*/
#define DATA_LENGTH 0b1001011

//predetermined constants
#define PACKET_SIZE 256
#define BYTE_SIZE 8

//useful constant
#define TRUE 1

//function declaration
void generatePacket(uint8_t packet[]);

void setup()
{
  //creates the array in which the packet will be stored
  uint8_t packet[PACKET_SIZE] = {0};

  /*arduino set up*/
  Serial.begin(115200);

  //creates the packet that must be sent
  generatePacket(packet);

  //array index can be changed: allows to check what any indexed packet is
  Serial.println(packet[0], BIN);

  //serial communication: sends packet over as bytes and confirms that it has been sent
  //NOTE: this has not been confirmed to work yet
  if (Serial.available() == TRUE)
  {
    Serial.write(packet, PACKET_SIZE);
    Serial.println("Packet has been sent");
  }
  /*end things*/
  Serial.end();
}

/*
 * Parameter: packet - array to modify with correct bytes
 * Creates data for packet
 */
void generatePacket(uint8_t packet[])
{
  //assembles the first byte from the packet start, address, and command/data flag
    packet[0] =
          (START << 4) | (ADDRESS << 1) | DATA_FLAG;

   //assigns the second byte
    packet[1] = DATA_LENGTH;

    //assigns arbitrary values to the payload field, for testing purposes
    int i;
    for (i = 2; i < PACKET_SIZE; i++)
    {
      packet[i] = 0xff; //this value may be modified to any between 0 and ff inclusive for 
//testing purposes
    }
}

//left here because it somehow would not compile without
void loop()
{

}

