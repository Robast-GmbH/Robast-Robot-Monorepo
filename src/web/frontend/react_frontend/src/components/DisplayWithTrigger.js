import React from 'react';
import Popup from './Popup';
import React from 'react';
import Popup from 'reactjs-popup';
import 'reactjs-popup/dist/index.css';


const DisplayWithTrigger = ({ onClick, logo, styleIn }) => {

        return (
                <Popup trigger={<button> Trigger</button>} position="center">
                        <div>Popup content here !!</div>
                </Popup>
        );
}


export default DisplayWithTrigger;