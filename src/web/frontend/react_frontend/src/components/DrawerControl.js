import * as React from 'react';
import PropTypes from 'prop-types'

import Stack from '@mui/material/Stack';

import Drawers from './Drawers';
import RobastRobotFront from './RobastRobotFront'
import ButtonPopUp from './ButtonPopup';
import RenameDrawer from './RenameDrawer';
import RefillDrawers from './RefillDrawers';




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
 
  return (
    <>
      <Stack direction="row">
        { /** Bild */}
        <Stack mr="80px">
          <RobastRobotFront/>  

          <Stack direction="row" spacing={2} id="drawer_controlls">
            {console.log(user)}
            {user.admin? <ButtonPopUp name="rename_drawer" caption="Umbennen" popUp={popUpModalRename(renameDrawer, drawers)} />:""}
            <ButtonPopUp name="refill_drawer" caption="Leer/ Refill" popUp={popUpModalRefill(toggleEmpty, drawers)}/>
          </Stack>
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