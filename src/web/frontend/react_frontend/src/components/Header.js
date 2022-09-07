import Button from './Button'
import { useLocation } from 'react-router-dom'
import DisplayAnImage from './DisplayAnImage';
import Logo from '../imgs/Logo.png';


const Header = ({ onAdd, showAdd, labelOpen, labelClose }) => {

        const location = useLocation()
        return (
                <header className='header'>
                        
                        <DisplayAnImage logo={Logo}></DisplayAnImage>
                        
                </header>
        )
}

Header.defaultProps = {
        lable: 'Robast Logo',
        //labelClose: 'Close',
        //labelOpen: 'View Order',
}

export default Header
/*{location.pathname === '/' && (<Button
                                color={showAdd ? 'red' : 'green'}
                                text={showAdd ? labelClose : labelOpen}
                                onClick={onAdd}
                        />)} */