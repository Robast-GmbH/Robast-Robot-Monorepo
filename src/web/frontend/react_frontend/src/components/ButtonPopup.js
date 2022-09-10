import PropTypes from 'prop-types'
import SimpleMap from './SimpleMap';
import MapPositions from './MapPositions';
import AddMapPosition from './AddMapPosition';
import Button from '@mui/material/Button';
import Popup from 'reactjs-popup';
import { useState, useEffect } from 'react'





export default function ButtonPopUp({name, caption,  popUp }) {
  const [popupIsOpen, setPopupIsOpen] = useState(false)
  const closeModal = () => setPopupIsOpen(false);
  const openModal = () => setPopupIsOpen(true);
  
  return (
            <div id= {name}>
             
              
              <Button id={name+"_executor"} class="NiceButton" variant="contained" onClick={openModal} >{caption}</Button>
              
              <Popup open={popupIsOpen} closeOnDocumentClick onClose={closeModal}>
              <>
              {popUp}
            
              </>
            </Popup>

            </div>
          );
}ButtonPopUp.propTypes = {
  name: PropTypes.string,
  caption: PropTypes.string,
  popUp: PropTypes.func,
}