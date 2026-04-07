inline bool commandLibrary(char device, char recvd, uint16_t argument = -1)
{
    // std::string rec = recvd;

    if (device == 'e') // engine or motor, doesnt handle command parsing logic, receives commands internally
    {
        if (recvd == 'B')
        {
            ESP_LOGI("serial", "LAMP ON");
        }
        else if (recvd == 'A')
        {
            ESP_LOGI("serial", "LAMP OFF");
        } else if (recvd == 'f')
        {
            xQueueSend(q_singleFrame, &recvd, 2);
            if (argument!=-1)
            {

            }
            ESP_LOGI("serial", "singleFrameFWD");


        } else if (recvd == 'r')
        {

            xQueueSend(q_singleFrame, &recvd, 2);
            if (argument!=-1)
            {
                
            }
            ESP_LOGI("serial", "singleFrameREV");



        }
    }
    // } else if (recvd == 'g')
    // {
    //             ESP_LOGI("serial", "Single Frame FWD");

    // } else if (recvd == 'h')
    // {
    //     ESP_LOGI("serial", "Single Frame REV");
    // } else if (recvd == 'F')
    // {
    //     ESP_LOGI("serial", "Motor FWD");

    // } else if (recvd == 'R')
    // {
    //     ESP_LOGI("serial", "Motor REV");
    // }

    return true;
}

// inline void commandString(char received) {
//     if (received[0] == '<')
//     {
//         char* strtokIndx;
//         strtokIndx = strtok(&received, ",");
//         uint8_t commandTemp = (uint8_t)*strtokIndx;
//         strtokIndx = strtok(NULL, ">");
//         uint16_t valueTemp = (uint16_t)*strtokIndx;

//                 ESP_LOGI("serial", "term 1 %u", commandTemp);
//                 ESP_LOGI("serial", "term 2 %u", valueTemp);

//         // commandLibrary((uint8_t)commandTemp, valueTemp);

//     }
// }