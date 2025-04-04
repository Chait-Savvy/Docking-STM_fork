#include "telecommand.h"

extern HAL_UART WIFI_1;

static int last_sign[4] = {1,1,1,1};
static pid ctrl[4];
static data_current_ctrl tx;


bool telecommand_thread::parse_tcmd(const char *msg, tcommand_t *cmd, float *data)
{
  // Check if the message starts with TCMD_START_CHAR and ends with TCMD_STOP_CHAR
  if (msg[0] != TELECOMMAND_START || msg[strlen(msg) - 1] != TELECOMMAND_STOP)
  {
    return false; // Invalid message format
  }

  // Find the position of the delimiter
  const char *delimiter_pos = strchr(msg, TELECOMMAND_DELIMITTER);
  if (!delimiter_pos)
  {
    return false; // Delimiter not found
  }

  // Extract and validate the command
  int command_int = 0;
  const char *ptr = msg + 1; // Start after TCMD_START_CHAR
  while (ptr < delimiter_pos)
  {
    if (*ptr >= '0' && *ptr <= '9')
    {
      command_int = command_int * 10 + (*ptr - '0'); // Convert char to int
    }
    else
    {
      return false; // Invalid character in command
    }
    ptr++;
  }

  // Validate the command range using TCMD_LENGTH
  if (command_int < 0 || command_int >= TCMD_LENGTH)
  {
    return false; // Invalid command
  }

  // Extract and convert the data
  float data_value = 0.0f;
  ptr = delimiter_pos + 1; // Start after TCMD_DELIMITER
  bool decimal_found = false;
  float decimal_place = 0.1f;

  while (*ptr != TELECOMMAND_STOP && *ptr != '\0')
  {
    if (*ptr >= '0' && *ptr <= '9')
    {
      if (decimal_found)
      {
        data_value += (*ptr - '0') * decimal_place;
        decimal_place *= 0.1f;
      }
      else
      {
        data_value = data_value * 10.0f + (*ptr - '0');
      }
    }
    else if (*ptr == '.')
    {
      decimal_found = true;
    }
    else
    {
      return false; // Invalid character in data
    }
    ptr++;
  }

  // Assign the parsed values
  *cmd = static_cast<tcommand_t>(command_int);
  *data = data_value;

  return true; // Success
}

double telecommand_thread::sign(const double in)
{
  if (in < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}

float telecommand_thread::pid_ctrl_magnets(int idx, float data)
{
  static float pwm[4] = {0,0,0,0};
  // float pwm;
  tx.i[0] = tx.i[1] = tx.i[2] = tx.i[3] = 0.0;
  magnet::get_current(tx.i); // Feedback measurements
  for(uint8_t i = 0; i < 4; i++)
    {
      tx.i[i] = last_sign[i] * tx.i[i]; // Signed current
      float error = data - tx.i[i];
      pwm[i] = ctrl[i].update(error, 20 / 1000.0);
      last_sign[i] = sign(pwm[i]); // Store sign
    }
  return pwm[idx]; 
}

bool telecommand_thread::execute_command(tcommand_t cmd, float data, tcmd_t *tcmd)
{
  tcmd->idx = cmd;
  tcmd->data = data;
  return true;
}

void telecommand_thread::init()
{
     WIFI_1.init(115200); 
     magnet::init();
     for(uint8_t i = 0; i < 4; i++)
     {
       ctrl[i].set_kp(PID_CURRENT_KP);
       ctrl[i].set_ki(PID_CURRENT_KI);
       ctrl[i].set_control_limits(PID_CURRENT_UMIN, PID_CURRENT_UMAX);
     }
//   led::init_far();
//   led::init_near();
}

void telecommand_thread::run()
{
  // while(true)
  // {
  //   // WIFI_1.suspendUntilDataReady();
  //   // decode_command(rx_buffer);
  //   // PRINTF("\n%c\n", rx_buffer);
  // }
  // TIME_LOOP(THREAD_TCMD_START * MILLISECONDS, TIME_PERIOD_PER_THREAD * MILLISECONDS)
  // {
    
    size_t rxlen = WIFI_1.read(tcmd_msg, sizeof(tcmd_msg));
    
    PRINTF("\n Inside TCMD thread, rcvd msg size : %d ",rxlen);
    if (rxlen > 0)
    {
      PRINTF(tcmd_msg);
      tcommand_t cmd;
      float data;

      if (parse_tcmd(tcmd_msg, &cmd, &data))
      {
        if(execute_command(cmd, data, &tcmd_data))
        {
          if(Magnet_safety_flag)
          {
            switch(cmd)
          {
            case TCMD_APPROACH:
              	magnet::actuate(MAGNET_IDX_0,pid_ctrl_magnets(0,data));
                magnet::actuate(MAGNET_IDX_1,pid_ctrl_magnets(1,data));
                magnet::actuate(MAGNET_IDX_2,pid_ctrl_magnets(2,data));
                magnet::actuate(MAGNET_IDX_3,pid_ctrl_magnets(3,data));
                PRINTF("APPROACH with force : %0.2f",data);
                break;
            case TCMD_ALLIGN_CW:
                magnet::stop(MAGNET_IDX_0);
                magnet::stop(MAGNET_IDX_3);
                magnet::actuate(MAGNET_IDX_1,pid_ctrl_magnets(1,data));
                magnet::actuate(MAGNET_IDX_2,pid_ctrl_magnets(2,data));
                PRINTF("ALLIGN CW with force : %0.2f",data);
                break;
            case TCMD_ALLIGN_CCW:
              	magnet::actuate(MAGNET_IDX_0,pid_ctrl_magnets(0,data));
                magnet::stop(MAGNET_IDX_1);
                magnet::stop(MAGNET_IDX_2);
                magnet::actuate(MAGNET_IDX_3,pid_ctrl_magnets(3,data));
                PRINTF("ALLIGN CCW with force : %0.2f",data);
                break;
            case TCMD_DOCKING_CW:
                magnet::stop(MAGNET_IDX_0);
                magnet::stop(MAGNET_IDX_3);
                magnet::actuate(MAGNET_IDX_1,pid_ctrl_magnets(1,data));
                magnet::actuate(MAGNET_IDX_2,pid_ctrl_magnets(2,data));
                PRINTF("Docking CW with force : %0.2f",data);
                break;   
            case TCMD_DOCKING_CCW:
                magnet::actuate(MAGNET_IDX_0,pid_ctrl_magnets(0,data));
                magnet::stop(MAGNET_IDX_1);
                magnet::stop(MAGNET_IDX_2);
                magnet::actuate(MAGNET_IDX_3,pid_ctrl_magnets(3,data));
                PRINTF("Docking CCW with force : %0.2f",data);
                break;
            case TCMD_MAGNETS_ON_OFF_FLAG : 
                magnet::stop(MAGNET_IDX_ALL);
                PRINTF("MAGNETS ON/OFF : %0.2f",data);
                break;              
          }

          } 
          
        }
      }
    }
  // }
}

// telecommand_thread tamariw_telecommand_thread("telecommand_thread", THREAD_PRIO_TELECOMMAND);
