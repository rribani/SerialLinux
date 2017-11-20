#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

int main()
{
  int fd;
  fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY);
  if(fd == 1)
     printf("\n  Error! in Opening ttyACM0\n");
  else
     printf("\n  ttyACM0 Opened Successfully\n");

  struct termios SerialPortSettings;	/* Create the structure                          */

  tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

  cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
  cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

  SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
  SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
  SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
  SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

  SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
  SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
  
  
  SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
  SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

  SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

  if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
      printf("\n  ERROR ! in Setting attributes");
  else
                  printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");
    
        /*------------------------------- Write data to serial port -----------------------------*/

  //char write_buffer[] = "<ÿÿÿÿÿÿÿÿÿÿ>";	/* Buffer containing characters to write into port	     */	
  char write_buffer[] = "<>";
  int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port */ 

  bytes_written = write(fd,write_buffer,sizeof(write_buffer));/* use write() to send data to port                                            */
                    /* "fd"                   - file descriptor pointing to the opened serial port */
                    /*	"write_buffer"         - address of the buffer containing data	            */
                    /* "sizeof(write_buffer)" - No of bytes to write                               */	
  printf("\n  %s written to ttyACM0",write_buffer);
  printf("\n  %d Bytes written to ttyACM0", bytes_written);
  printf("\n +----------------------------------+\n\n");

  close(fd);/* Close the Serial port */
  
  return 0;
}
