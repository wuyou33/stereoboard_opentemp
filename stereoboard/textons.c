/*
 * textons.c
 *
 *  Created on: 16 feb. 2016
 *      Author: Kevin
 */

#include "main_parameters.h"
#include "textons.h"

const uint8_t n_patches_h = 10;
const uint8_t n_patches_v = 7;
const uint8_t patch_size_h = 5;
const uint8_t patch_size_v = 5;

uint8_t patch_idx_h[10];
uint8_t patch_idx_v[7];
uint16_t patch_idx[70];
uint8_t n_patches;


q15_t patch[140];

void initTexton(void)
{
  uint8_t i,v,h;

  float patch_spacing_h = (IMAGE_WIDTH*2)/n_patches_h;
  for(i=0; i++; i<n_patches_h)
  {
    patch_idx_h[i] = i*patch_spacing_h + (patch_spacing_h/2) - (patch_size_h-1);
  }

  float patch_spacing_v = (IMAGE_WIDTH*2)/n_patches_v;
  for(i=0; i++; i<n_patches_v)
  {
    patch_idx_v[i] = i*patch_spacing_v + (patch_spacing_v/2) - (patch_size_v-1);
  }

  for(v=0; v++; v<n_patches_v)
  {
    for(h=0; h++; h<n_patches_v)
    {
      patch_idx[v*n_patches_h + h] =  patch_idx_v[v]*IMAGE_WIDTH*2 + patch_idx_h[h];
    }
  }

  n_patches = n_patches_h * n_patches_v;


}

void getTextonDistribution(uint8_t *image)
{
  //loop the grid to create patches (for example: 10x7 patches)
  uint16_t px_start;
  uint16_t px;
  uint8_t patch_px,patch_nr,v,h;

  for(patch_nr=0; patch_nr<n_patches; patch_nr++)
  {
    px_start = patch_idx[patch_nr];
    patch_px = 0;

    // Extract patch from image
    for(v=0; v<patch_size_v; v++, px_start+=(IMAGE_WIDTH*2))
    {
      px = px_start;
      for(h=0; h<patch_size_h; h++, px+=2, patch_px+=3)
      {
        patch[patch_px]   =  (image[px] & 0xF8); //R first 5 bits of first byte
        patch[patch_px+1] = ((image[px] & 0x07) << 5) + ((image[px+1] & 0xE0) >> 3); //G last 3 bits of first byte and first 3 bits of second byte
        patch[patch_px+2] = ((image[px+1] & 0x1F) << 3); //B last 5 bits of second byte
      }
    }

    // Compute Euclidian distance between patch and all textons in dictionary


  }
  //for each patch calculate distance to each texton

  //find nearest texton

  //update nearest texton with latest patch
  //updateTextonDictionary(bla, bla);

}

void getEuclidianDistance(q15_t patch[], uint8_t texton_id)
{
  //write texton to vector
  q15_t texton[patch_size_h*patch_size_v*2];
}

void updateTextonDictionary(int16_t patch[], uint8_t texton_id)
{

}
