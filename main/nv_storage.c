#include "nv_storage.h"
#include "nvs_flash.h"
#include "typedefs.h"

void save_config(nav_config_t *cfg)
{
    nvs_handle_t nvm_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
    } 
    else 
    {
        // Struct'i NVM'e yazma
        ret = nvs_set_blob(nvm_handle, "config", cfg, sizeof(nav_config_t));
        if (ret != ESP_OK) 
        {
            printf("Error (%s) writing data to NVS!\n", esp_err_to_name(ret));
        } 
        else 
        {
            printf("Data written to NVS!\n");
            nvs_commit(nvm_handle);
        }

        nvs_close(nvm_handle);
    }
}


void read_config(nav_config_t *cfg)
{
    nvs_handle_t nvm_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
    }
    else
    {
        size_t required_size;
        ret = nvs_get_blob(nvm_handle, "config", NULL, &required_size);
        if (ret == ESP_OK && required_size == sizeof(nav_config_t)) 
        {
            ret = nvs_get_blob(nvm_handle, "config", cfg, &required_size);
            if (ret != ESP_OK) 
            {
                printf("Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
            } 
        } 
        else 
        {
            printf("Error (%s) reading data size from NVS!\n", esp_err_to_name(ret));
        }

        nvs_close(nvm_handle);
    }
}

void print_config(nav_config_t cfg)
{
    float *p = (float *)&cfg;
    for (int i = 0; i < 11; i++) 
    {
        printf("s.%c = %f\n", 'a' + i, p[i]);
    }
}