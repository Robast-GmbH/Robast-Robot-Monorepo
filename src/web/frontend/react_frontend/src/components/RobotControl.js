import PropTypes from 'prop-types';
import { useState, useEffect } from 'react';
import SimpleMap from './SimpleMap';
import MapPositions from './MapPositions';
import ButtonPopUp from './ButtonPopup';
import AddMapPosition from './AddMapPosition';
import IconButton from '@mui/material/IconButton';
import Button from '@mui/material/Button';
import Stack from '@mui/material/Stack';

import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import HomeIcon from '@mui/icons-material/Home';

const popupModal= (addMapPosition)=> {return ( 
<div>
  <h1>Add a Waypoint</h1>
  {<AddMapPosition onAdd={addMapPosition}/>}
</div>
  ); }
  

export default function RobotControl({mapPositions, sendGoal,  addMapPosition, robotStatusChange}) {
  
  return (
            <div class = "RobotControl" >
              <div class ="ControlPanel">
             { /* <Stack direction="row" spacing={1}>
                  <IconButton id="Pause"  variant="contained" color="error" size	="large" onClick={(prop)=>{robotStatusChange(2)}} >
                    <PauseIcon/>
                  </IconButton>
                  <IconButton id="Resume" variant="contained" color= "success" size	="large"  onClick={(prop)=>{robotStatusChange(1)}} >
                    <PlayArrowIcon/>
                  </IconButton>
                  <IconButton id="Home" variant="contained" color= "info" size	="large"  onClick={(prop)=>{robotStatusChange(3)}} >
                    <HomeIcon/>
                  </IconButton>
                </Stack> */}
               <Stack direction="row" spacing={1}>
                <Button id="Pause"  variant="contained" color="primary" startIcon={<PauseIcon />} onClick={(prop)=>{robotStatusChange(2)}} >
                  Pause
                </Button>
                <Button id="Resume"  variant="contained" color="primary" startIcon={<PlayArrowIcon />} onClick={(prop)=>{robotStatusChange(1)}} >
                Resume
                </Button>
                <Button id="Resume"  variant="contained" color="primary" startIcon={<HomeIcon />} onClick={(prop)=>{robotStatusChange(3)}} >
                Home
                </Button>
                </Stack>
                
              </div>

              <SimpleMap/>
              <Stack direction="row" spacing={1}>
              {mapPositions.length > 0 ? (
                <MapPositions mapPositions={mapPositions} sendGoal={sendGoal}/>
              ):('')
              }
              </Stack>
            <br></br>  
            <ButtonPopUp name="AddPosition" caption="Neuer Punkt" popUp= {popupModal(addMapPosition)}/>
             

            </div>
          );
}
