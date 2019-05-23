package org.firstinspires.ftc.teamcode.util;

/**
 * Created by thomas on 2018-2-23.
 */

import java.util.LinkedHashSet;
import java.util.Properties;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Set;
import java.util.Vector;


public class OrderedProperties extends Properties {
    private static final long serialVersionUID = -4627607243846121965L;
    private final LinkedHashSet<Object> keys = new LinkedHashSet<Object>();

//     public Enumeration keys() {
// //     return Collections.<Object> enumeration(keys);
//     //     follow is by alphabet
//         Enumeration keysEnum = super.keys();
//         Vector<String> keyList = new Vector<String>();
//         while(keysEnum.hasMoreElements()){
//             keyList.add((String)keysEnum.nextElement());
//         }
//         Collections.sort(keyList);
//         return keyList.elements();

//     }

    public Object put(Object key, Object value) {
        keys.add(key);
        return super.put(key, value);
    }

    public synchronized Object remove(Object key) {
        keys.remove(key);
        return super.remove(key);
    }

    public Set<Object> keySet() {
        return keys;
    }

    public Set<String> stringPropertyNames() {
        Set<String> set = new LinkedHashSet<String>();
        for (Object key : this.keys) {
            set.add((String) key);
        }
        return set;
    }
}



