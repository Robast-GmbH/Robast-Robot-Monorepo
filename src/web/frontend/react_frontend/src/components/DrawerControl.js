import * as React from 'react';
import PropTypes from 'prop-types'

import Stack from '@mui/material/Stack';

import Drawers from './Drawers';
import RobastRobotFront from './RobastRobotFront'
import ButtonPopUp from './ButtonPopup';
import RenameDrawer from './RenameDrawer';
import RefillDrawers from './RefillDrawers';
import { useTheme } from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';




const popUpModalRename= (renameDrawer, drawers)=> 
{
  return ( 
            <>
                {<RenameDrawer renameDrawer={renameDrawer} drawers={drawers}/>}
            </> 
     );
  }

const popUpModalRefill= ( toggleEmpty, drawers )=>
{
  return(
    <>
        {<RefillDrawers drawers= {drawers} toggleEmpty={toggleEmpty} />}
    </>
  );
}

const DrawerControl= ({drawers, user, openDrawer, renameDrawer, toggleEmpty})=> {
  const theme = useTheme();
  const matches = useMediaQuery(theme.breakpoints.up('sm'));
  return (
    <>
      <Stack direction={ matches ? "row":"column-reverse"}  spacing={4}>
        { /** Bild */}
        <Stack  mr={ matches ? "80px":"0px"} direction="column" >
          <RobastRobotFront/>  

          { <Stack direction="row" spacing={1} id="drawer_controlls" orientation="center" >
            {user.admin? <ButtonPopUp name="rename_drawer" caption="Umbennen" popUp={popUpModalRename(renameDrawer, drawers)} />:""}
            <ButtonPopUp name= {user.admin? "admin_refill_drawer" :"refill_drawer"} caption="Leer / Befüllen" popUp={popUpModalRefill(toggleEmpty, drawers)}/>
          </Stack>}
        </Stack>

        { /** Drawer */}
        <Stack id="drawer_list">
          <Drawers drawers={drawers} openDrawer={openDrawer} toggleEmpty= {toggleEmpty} user= {user}/>
        </Stack>
      </Stack>
    </>
    
  )
}
DrawerControl.propTypes = {
  drawer: PropTypes.array,
  user: PropTypes.object,
  openDrawer: PropTypes.func,
  renameDrawer: PropTypes.func,
  isEmpty: PropTypes.func,
  getDrawers: PropTypes.func,
}
export default DrawerControl