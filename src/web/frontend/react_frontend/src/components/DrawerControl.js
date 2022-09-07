import * as React from 'react';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import Box from '@mui/material/Box';
import Drawers from './Drawers';
import RobastRobotFront from './RobastRobotFront'
import RefillButton from './RefillButton'



export default function DrawerControl({drawers, openDrawer}) {
  console.log({drawers})
  return (
    <div>
      <RobastRobotFront/>
      <Drawers drawers={drawers} openDrawer={openDrawer}/>
      
      <RefillButton/>
      
    </div>
  )
}