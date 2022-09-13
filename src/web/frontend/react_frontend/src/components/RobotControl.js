import PropTypes from 'prop-types';
import { useState, useEffect } from 'react';
import SimpleMap from './SimpleMap';

import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import HomeIcon from '@mui/icons-material/Home';
//import IconButton from '@mui/material/IconButton';

import ButtonPopUp from './ButtonPopup';
import AddMapPosition from './AddMapPosition';

import Button from '@mui/material/Button';
import Stack from '@mui/material/Stack';

import { useTheme } from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';

import GoalSelector from './GoalSelector';

const popupModal= (addMapPosition)=> {return ( 
<>
  {<AddMapPosition onAdd={addMapPosition}/>}
</>
  ); }
  

export default function RobotControl({user, mapPositions, sendGoal,  addMapPosition, robotStatusChange}) {
  const theme = useTheme();
  const matches = useMediaQuery(theme.breakpoints.up('sm'));
  return (
          <Stack>
          
              
               <Stack direction="row" spacing={1}   justifyContent="center" >
                <Button id="Pause"  variant="contained" color="primary" startIcon={<PauseIcon />} onClick={(prop)=>{robotStatusChange(2)}} >
                  {(matches?"Pause": "")} 
                </Button>
                <Button id="Resume"  variant="contained" color="primary" startIcon={<PlayArrowIcon />} onClick={(prop)=>{robotStatusChange(1)}} >
                 {(matches?"Resume": "")} 
                </Button>
                <Button id="Resume"  variant="contained" color="primary" startIcon={<HomeIcon />} onClick={(prop)=>{robotStatusChange(3)}} >
                  {(matches?"Home": "")} 
                </Button>
                </Stack>

              <SimpleMap/>
              <Stack direction="row" justifyContent= "space-between" >
                {user.admin?(<GoalSelector mapPositions={mapPositions} sendGoal= {sendGoal}/>):("")}
                {user.admin?(<ButtonPopUp name="AddPosition" caption="Neuer Punkt" popUp= {popupModal(addMapPosition)}/>):("")}
              </Stack>
            </Stack>
          );
}
